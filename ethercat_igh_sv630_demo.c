/*****************************************************************************
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>     /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h>    /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/** Task period in ns. */
#define NSEC_PER_SEC (1000000000L)

#define FREQUENCY 1000
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define SHIFT_NS (NSEC_PER_SEC / FREQUENCY / 4)

#define CLOCK_TO_USE CLOCK_MONOTONIC

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

#define TASK_FREQUENCY 10 /*Hz*/

// #define MEASURE_TIMING

/****************************************************************************/

/* Constants */

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state = {};

/****************************************************************************/

// process data
static uint8_t *domain_pd = NULL;

// alias, position
#define BusCouplerPos 1, 0

/* Master 0, Slave 0, "InoSV630N"
 * Vendor ID:       0x00100000
 * Product code:    0x000c0112
 * Revision number: 0x00010000
 */
// vendor_id, product_code
#define InoSV630N 0x00100000, 0x000c0112

// offsets for PDO entries 定义PDO条⽬的偏移量
static unsigned int Status_Word;
static unsigned int Control_Word;
static unsigned int Target_Position;
static unsigned int Position_Actual_Value;

static unsigned int counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

const static ec_pdo_entry_reg_t domain1_regs[] = {
    {BusCouplerPos, InoSV630N, 0x6041, 0x00, &Status_Word},
    {BusCouplerPos, InoSV630N, 0x6040, 0x00, &Control_Word},
    {BusCouplerPos, InoSV630N, 0x607a, 0x00, &Target_Position},
    {BusCouplerPos, InoSV630N, 0x6064, 0x00, &Position_Actual_Value},
    // {}
};

/****************************************************************************/

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x607a, 0x00, 32}, /* Target Position */
    {0x60b8, 0x00, 16},
    {0x60fe, 0x01, 32}, /* Digital outputs */
    {0x603f, 0x00, 16},
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6064, 0x00, 32}, /* Position Actual Value */
    {0x6077, 0x00, 16},
    {0x60f4, 0x00, 32},
    {0x60b9, 0x00, 16},
    {0x60ba, 0x00, 32},
    {0x60bc, 0x00, 32},
    {0x60fd, 0x00, 32}, /* Digital inputs */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1701, 4, slave_0_pdo_entries + 0}, /* R0PDO */
    {0x1b01, 9, slave_0_pdo_entries + 4}, /* T0PDO */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}};

uint16_t ctrl_word[] = {0x06, 0x07, 0x0f};
uint16_t clear_word = 0x80;

/****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
    {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state)
    {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_ana_in, &s);

    if (s.al_state != sc_ana_in_state.al_state)
    {
        printf("AnaIn: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_ana_in_state.online)
    {
        printf("AnaIn: %s.\n", s.online ? "online" : "offline");

        // if (s.online)
        // {
        //     EC_WRITE_U16(domain_pd + Control_Word, clear_word);
        // }
    }
    if (s.operational != sc_ana_in_state.operational)
    {
        printf("AnaIn: %soperational.\n", s.operational ? "" : "Not ");
    }

    sc_ana_in_state = s;
}

/****************************************************************************/
uint16_t raw_value = 0;

void cyclic_task()
{
    struct timespec time;

    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    raw_value = EC_READ_U16(domain_pd + Status_Word);
    raw_value = raw_value & 0x03ff;
    uint16_t ctr_value = EC_READ_U16(domain_pd + Control_Word);

    if (raw_value != 0)
    {
        printf("raw_value: 0x%04x, ctr_value: 0x%04x\n", raw_value, ctr_value);

        static int i = 0;
        i++;

        if (raw_value == 0x0218)
        {
            EC_WRITE_U16(domain_pd + Control_Word, clear_word);
        }

        // 伺服初始化状态机
        if ((raw_value == 0x0250) && (ctr_value == 0))
        {
            EC_WRITE_U16(domain_pd + Control_Word, ctrl_word[0]);
        }
        if (raw_value == 0x0231 && (ctr_value == 0x06))
        {
            EC_WRITE_U16(domain_pd + Control_Word, ctrl_word[1]);
        }
        if (raw_value == 0x0233 && (ctr_value == 0x07))
        {
            EC_WRITE_U16(domain_pd + Control_Word, ctrl_word[2]);

            EC_WRITE_S32(domain_pd + Target_Position, i * 50);
        }

        // 设置位置
        if ((raw_value == 0x0237) || (raw_value == 0x0238))
        {
            EC_WRITE_S32(domain_pd + Target_Position, i * 50);
        }
    }

    // check process data state
    check_domain1_state();

    if (counter)
    {
        counter--;
    }
    else
    {
        counter = FREQUENCY;

        // check for master state (optional)
        check_master_state();
        // check for slave configuration state(s) (optional)
        check_slave_config_states();
    }

    clock_gettime(CLOCK_TO_USE, &time);
    ecrt_master_application_time(master, TIMESPEC2NS(time));
    // 同步参考时钟
    ecrt_master_sync_reference_clock(master);
    // 同步所有从站时钟
    ecrt_master_sync_slave_clocks(master);

    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC)
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    }
    else
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/****************************************************************************/

int main(int argc, char **argv)
{
    struct timespec wakeup_time;
    int ret = 0;

    master = ecrt_request_master(0);
    if (!master)
    {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        return -1;
    }

    if (!(sc_ana_in = ecrt_master_slave_config(
              master, BusCouplerPos, InoSV630N)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    printf("Configuring PDOs...\n");
    // 设置PDOS，利⽤sc_ana_in、EC_END、对⻬信息
    if (ecrt_slave_config_pdos(sc_ana_in, EC_END, slave_0_syncs))
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // Create configuration for bus coupler
    // 在域domain中注册PDOs
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    // 同步时钟
    // configure SYNC signals for this slave
    // ecrt_slave_config_dc(sc_ana_in, 0x0700, PERIOD_NS, 4400000, 0, 0);
    // ecrt_slave_config_dc(sc_ana_in, 0x0700, 0, 0, 0, 0);

    ecrt_slave_config_dc(sc_ana_in, 0x0300, PERIOD_NS, SHIFT_NS, 0, 0); // 根据设备xml设置

    // 激活master
    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        return -1;
    }

    // 把域的数据地址给到domain_pd
    if (!(domain_pd = ecrt_domain_data(domain1)))
    {
        return -1;
    }

    printf("domain_pd :%p\n", domain_pd);

    /* Set priority */
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.\n", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }

    /* Lock memory */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();

    printf("Starting RT task with dt=%lu ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_TO_USE, &wakeup_time);

#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif

    while (1)
    {
        wakeup_time = timespec_add(wakeup_time, cycletime);
        ret = clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME,
                              &wakeup_time, NULL);
        if (ret)
        {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }
        // Write application time to master
        // It is a good idea to use the target time (not the measured time) as
        // application time, because it is more stable.
        // ecrt_master_application_time(master, TIMESPEC2NS(wakeup_time));

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeup_time, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns)
        {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns)
        {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns)
        {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns)
        {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns)
        {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns)
        {
            exec_min_ns = exec_ns;
        }
#endif

        cyclic_task();

#ifdef MEASURE_TIMING
        // output timing stats
        printf("period     %10u ... %10u\n",
               period_min_ns, period_max_ns);
        printf("exec       %10u ... %10u\n",
               exec_min_ns, exec_max_ns);
        printf("latency    %10u ... %10u\n",
               latency_min_ns, latency_max_ns);
        period_max_ns = 0;
        period_min_ns = 0xffffffff;
        exec_max_ns = 0;
        exec_min_ns = 0xffffffff;
        latency_max_ns = 0;
        latency_min_ns = 0xffffffff;
#endif

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif
    }

    // 解锁内存
    // munlockall();

    return ret;
}

/****************************************************************************/
