/* main.c
   Tahap 9: Implementasi Statistik Simulasi Lengkap
   Problem 2.38 "A car-rental system" (Averill M. Law)
   Menggunakan pustaka C SIMLIB
*/

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "simlib.h"

/* ========================================================================== */
/* 1. CONSTANTS & PARAMETERS                                                  */
/* ========================================================================== */

/* Waktu Simulasi: 80 jam = 288,000 detik */
#define SIMULATION_TIME   288000.0f
#define MIN_STOP_TIME     300.0f  /* 5 menit */
#define BUS_CAPACITY      20

/* Waktu Tempuh Rute (Detik) pada Kecepatan 30 mph */
#define TRAVEL_TIME_3_1   540.0f  /* 4.5 miles -> 9.0 min  */
#define TRAVEL_TIME_1_2   120.0f  /* 1.0 mile  -> 2.0 min  */
#define TRAVEL_TIME_2_3   540.0f  /* 4.5 miles -> 9.0 min  */

/* Rata-rata Waktu Antar-Kedatangan Penumpang (Detik) */
#define MEAN_ARRIV_1      (3600.0f / 14.0f)  /* ~257.14 s */
#define MEAN_ARRIV_2      (3600.0f / 10.0f)  /* 360.00 s  */
#define MEAN_ARRIV_3      (3600.0f / 24.0f)  /* 150.00 s  */

/* Probabilitas Tujuan dari Car Rental (Lokasi 3) */
#define PROB_DEST_TERM_1  0.583f

/* Waktu Unloading dan Loading per Orang (Detik) */
#define UNLOAD_MIN        16.0f
#define UNLOAD_MAX        24.0f
#define LOAD_MIN          25.0f
#define LOAD_MAX          35.0f

/* Alokasi Random Number Streams (1 s.d. 6) */
#define STREAM_ARRIV_1    1
#define STREAM_ARRIV_2    2
#define STREAM_ARRIV_3    3
#define STREAM_DEST_3     4
#define STREAM_UNLOAD     5
#define STREAM_LOAD       6

/* Identifikasi List SIMLIB */
#define LIST_Q1           1
#define LIST_Q2           2
#define LIST_Q3           3
#define LIST_BUS          4
/* Catatan: LIST_EVENT (25) sudah didefinisikan secara internal oleh simlibdefs.h */

/* Identifikasi Tipe Event Diskrit */
#define EVENT_ARRIVAL_1       1
#define EVENT_ARRIVAL_2       2
#define EVENT_ARRIVAL_3       3
#define EVENT_BUS_ARRIVAL     4
#define EVENT_END_UNLOADING   5
#define EVENT_END_LOADING     6
#define EVENT_BUS_DEPARTURE   7
#define EVENT_END_SIMULATION  8

/* ========================================================================== */
/* 2. DATA STRUCTURES & ACCUMULATORS                                          */
/* ========================================================================== */

/* Struktur Bus tunggal */
struct Bus {
    int   location;               /* Lokasi halte saat ini (1, 2, atau 3) */
    int   passenger_count;        /* Jumlah penumpang di dalam bus (0..20) */
    float arrival_time_at_stop;   /* Waktu saat bus tiba di halte saat ini */
};

static struct Bus bus;

/* Struktur sementara untuk menampung data penumpang yang sedang unloading */
struct UnloadRecord {
    float arrival_time;
    int   destination;
    int   origin;
    long  pid;
};

static struct UnloadRecord unloading_batch[BUS_CAPACITY + 1];
static int unloading_count = 0;

/* ID generator unik untuk melacak passenger */
static long next_passenger_id = 1;

/* Counter operasional */
static long total_arrivals[4] = {0, 0, 0, 0};
static long total_unloaded = 0;
static long total_loaded   = 0;

/* Counter bus stop untuk membatasi tampilan log debugging agar rapi */
static int bus_stop_count = 0;
#define MAX_STOPS_TO_PRINT 12

/* -------------------------------------------------------------------------- */
/* STATISTIK STRUKTUR                                                         */
/* -------------------------------------------------------------------------- */

/* Struktur untuk statistik observasi diskrit (Discrete Observations) */
struct DiscreteStat {
    long   count;
    double sum;
    double min;
    double max;
};

/* Struktur untuk statistik rata-rata terhadap waktu (Time-Average) */
struct TimeAvgStat {
    double area;
    double last_update;
    double max;
};

/* Global Accumulators */
static struct TimeAvgStat  stat_queue_len[4];    /* Indeks 1, 2, 3 untuk Q1, Q2, Q3 */
static struct DiscreteStat stat_queue_delay[4];  /* Indeks 1, 2, 3 untuk Loc 1, 2, 3 */
static struct TimeAvgStat  stat_bus_occupancy;   /* Time-average okupansi bus */
static struct DiscreteStat stat_stop_time[4];    /* Indeks 1, 2, 3 untuk Halte 1, 2, 3 */
static struct DiscreteStat stat_loop_time;       /* Loop time arrival-to-arrival at Loc 3 */

/* Time in System berdasarkan rute:
 * 0: 1 -> 3
 * 1: 2 -> 3
 * 2: 3 -> 1
 * 3: 3 -> 2
 */
static struct DiscreteStat stat_tis_route[4];
static struct DiscreteStat stat_tis_overall;

/* Pelacak waktu loop */
static float last_arrival_at_loc3 = 0.0f;
static int   arrival_at_loc3_count = 0;

/* ========================================================================== */
/* 3. STATISTIC HELPER FUNCTIONS                                              */
/* ========================================================================== */

void reset_discrete_stat(struct DiscreteStat *s)
{
    s->count = 0;
    s->sum   = 0.0;
    s->min   = FLT_MAX;
    s->max   = 0.0;
}

void record_discrete_stat(struct DiscreteStat *s, double value)
{
    s->count++;
    s->sum += value;
    if (value < s->min) {
        s->min = value;
    }
    if (value > s->max) {
        s->max = value;
    }
}

double get_discrete_avg(const struct DiscreteStat *s)
{
    if (s->count == 0) return 0.0;
    return s->sum / (double) s->count;
}

void reset_time_avg_stat(struct TimeAvgStat *s)
{
    s->area        = 0.0;
    s->last_update = 0.0;
    s->max         = 0.0;
}

void update_queue_stat(int loc)
{
    double current_len = (double) list_size[loc];
    double delta_t = sim_time - stat_queue_len[loc].last_update;

    stat_queue_len[loc].area += current_len * delta_t;
    stat_queue_len[loc].last_update = sim_time;
}

void update_bus_occupancy_stat(void)
{
    double current_occ = (double) bus.passenger_count;
    double delta_t = sim_time - stat_bus_occupancy.last_update;

    stat_bus_occupancy.area += current_occ * delta_t;
    stat_bus_occupancy.last_update = sim_time;
}

/* ========================================================================== */
/* 4. FUNCTION PROTOTYPES                                                     */
/* ========================================================================== */

void init_model(void);
void handle_passenger_arrival(int location);
void handle_bus_arrival(void);
void handle_end_unloading(void);
void handle_end_loading(void);
void handle_bus_departure(void);
void print_statistics(void);

/* ========================================================================== */
/* 5. PASSENGER ARRIVAL HANDLER                                               */
/* ========================================================================== */

void handle_passenger_arrival(int location)
{
    int dest;
    int list_id;
    float mean_interarrival;
    int stream_id;
    int event_type;
    long pid = next_passenger_id++;

    /* 1. Tentukan tujuan berdasarkan lokasi kedatangan */
    if (location == 1) {
        dest = 3; /* Terminal 1 selalu ke Car Rental */
        list_id = LIST_Q1;
        mean_interarrival = MEAN_ARRIV_1;
        stream_id = STREAM_ARRIV_1;
        event_type = EVENT_ARRIVAL_1;
    } else if (location == 2) {
        dest = 3; /* Terminal 2 selalu ke Car Rental */
        list_id = LIST_Q2;
        mean_interarrival = MEAN_ARRIV_2;
        stream_id = STREAM_ARRIV_2;
        event_type = EVENT_ARRIVAL_2;
    } else { /* location == 3 (Car Rental) */
        /* Pilih tujuan berdasarkan probabilitas 0.583 vs 0.417 */
        if (lcgrand(STREAM_DEST_3) < PROB_DEST_TERM_1) {
            dest = 1;
        } else {
            dest = 2;
        }
        list_id = LIST_Q3;
        mean_interarrival = MEAN_ARRIV_3;
        stream_id = STREAM_ARRIV_3;
        event_type = EVENT_ARRIVAL_3;
    }

    total_arrivals[location]++;

    /* 2. Simpan atribut passenger ke buffer transfer[] */
    transfer[1] = sim_time;             /* Arrival Time */
    transfer[2] = (float) dest;         /* Destination  */
    transfer[3] = (float) location;     /* Origin       */
    transfer[4] = (float) pid;          /* Passenger ID */

    /* 3. Update statistik area antrean SEBELUM penambahan elemen */
    update_queue_stat(location);

    /* 4. Masukkan passenger ke antrean FIFO lokasi bersangkutan */
    list_file(LAST, list_id);

    /* Update maximum queue length jika bertambah */
    if ((double) list_size[list_id] > stat_queue_len[location].max) {
        stat_queue_len[location].max = (double) list_size[list_id];
    }

    /* 5. Cetak log beberapa penumpang pertama */
    if (pid <= 6) {
        printf("[t=%9.2f s] PASSENGER ARRIVAL : Passenger %ld at Loc %d -> Dest %d (Queue: %d)\n",
               sim_time, pid, location, dest, list_size[list_id]);
    }

    /* 6. Jadwalkan kedatangan berikutnya dari lokasi yang sama */
    event_schedule(sim_time + expon(mean_interarrival, stream_id), event_type);
}

/* ========================================================================== */
/* 6. BUS ARRIVAL & UNLOADING HANDLERS                                        */
/* ========================================================================== */

void handle_bus_arrival(void)
{
    int original_on_bus;
    int i;
    float total_unloading_time = 0.0f;

    bus.arrival_time_at_stop = sim_time;
    bus_stop_count++;

    /* Evaluasi Bus Loop Time (Arrival-to-Arrival di Location 3) */
    if (bus.location == 3) {
        arrival_at_loc3_count++;
        if (arrival_at_loc3_count > 1) {
            double loop_time = sim_time - last_arrival_at_loc3;
            record_discrete_stat(&stat_loop_time, loop_time);
        }
        last_arrival_at_loc3 = sim_time;
    }

    if (bus_stop_count <= MAX_STOPS_TO_PRINT) {
        printf("\n-------------------------------------------------------------------\n");
        printf("[t=%9.2f s] BUS ARRIVAL at Location %d (Bus onboard: %d, Queue: %d)\n",
               sim_time, bus.location, bus.passenger_count, list_size[bus.location]);
    }

    /* Tentukan passenger mana di dalam bus yang destination-nya == bus.location */
    original_on_bus = list_size[LIST_BUS];
    unloading_count = 0;

    /* Pindai seluruh penumpang di bus secara aman menggunakan API SIMLIB */
    for (i = 0; i < original_on_bus; i++) {
        list_remove(FIRST, LIST_BUS);

        if ((int) transfer[2] == bus.location) {
            /* Penumpang ini tujuannya adalah halte ini -> ditandai untuk turun */
            unloading_batch[unloading_count].arrival_time = transfer[1];
            unloading_batch[unloading_count].destination  = (int) transfer[2];
            unloading_batch[unloading_count].origin       = (int) transfer[3];
            unloading_batch[unloading_count].pid          = (long) transfer[4];
            unloading_count++;

            /* Bangkitkan random unloading time independen per orang: Uniform(16, 24) */
            total_unloading_time += uniform(UNLOAD_MIN, UNLOAD_MAX, STREAM_UNLOAD);
        } else {
            /* Penumpang ini tujuannya di halte lain -> tetap berada di bus */
            list_file(LAST, LIST_BUS);
        }
    }

    if (bus_stop_count <= MAX_STOPS_TO_PRINT) {
        if (unloading_count > 0) {
            printf("[t=%9.2f s]   -> Unloading %d passenger(s), Duration = %.2f s (Finish at t=%.2f)\n",
                   sim_time, unloading_count, total_unloading_time, sim_time + total_unloading_time);
        } else {
            printf("[t=%9.2f s]   -> No passengers to unload (Duration = 0.00 s)\n", sim_time);
        }
    }

    /* Jadwalkan akhir proses unloading */
    event_schedule(sim_time + total_unloading_time, EVENT_END_UNLOADING);
}

void handle_end_unloading(void)
{
    int i;
    int available_seats;
    int queue_id;
    int m;
    float total_loading_time = 0.0f;

    /* 1. Proses data passenger yang selesai turun */
    if (unloading_count > 0) {
        /* Update time-average bus occupancy SEBELUM penumpang turun */
        update_bus_occupancy_stat();

        bus.passenger_count -= unloading_count;

        for (i = 0; i < unloading_count; i++) {
            total_unloaded++;

            /* Hitung Time in System: sim_time - arrival_time */
            double tis = (double)(sim_time - unloading_batch[i].arrival_time);
            int orig = unloading_batch[i].origin;
            int dest = unloading_batch[i].destination;

            /* Tentukan route index untuk statistik:
             * 0: 1 -> 3
             * 1: 2 -> 3
             * 2: 3 -> 1
             * 3: 3 -> 2
             */
            int r_idx = -1;
            if (orig == 1 && dest == 3) r_idx = 0;
            else if (orig == 2 && dest == 3) r_idx = 1;
            else if (orig == 3 && dest == 1) r_idx = 2;
            else if (orig == 3 && dest == 2) r_idx = 3;

            if (r_idx >= 0) {
                record_discrete_stat(&stat_tis_route[r_idx], tis);
            }
            record_discrete_stat(&stat_tis_overall, tis);

            if (bus_stop_count <= MAX_STOPS_TO_PRINT) {
                printf("[t=%9.2f s]   [UNLOADED] Passenger %ld (Origin %d -> Dest %d, TIS = %.2f s)\n",
                       sim_time, unloading_batch[i].pid, orig, dest, tis);
            }
        }
        unloading_count = 0; /* Reset batch unloading */
    }

    /* 2. Hitung kursi kosong yang tersedia */
    available_seats = BUS_CAPACITY - bus.passenger_count;
    queue_id = bus.location;

    /* 3. Tentukan berapa banyak passenger dari queue yang boleh naik */
    m = list_size[queue_id];
    if (m > available_seats) {
        m = available_seats;
    }

    if (bus_stop_count <= MAX_STOPS_TO_PRINT) {
        printf("[t=%9.2f s]   -> End Unloading. Available seats = %d, Queue = %d -> Loading %d passenger(s)\n",
               sim_time, available_seats, list_size[queue_id], m);
    }

    /* 4. Ambil m passenger dari kepala queue halte secara FIFO dan masukkan ke bus */
    for (i = 0; i < m; i++) {
        /* Update statistik queue SEBELUM passenger dikeluarkan */
        update_queue_stat(queue_id);

        /* Ambil penumpang terdepan dari queue */
        list_remove(FIRST, queue_id);

        /* Hitung Queue Delay: waktu saat ini (mulai boarding) - waktu kedatangan di antrean */
        double delay = (double)(sim_time - transfer[1]);
        record_discrete_stat(&stat_queue_delay[queue_id], delay);

        /* Update statistik bus occupancy SEBELUM penumpang masuk ke bus */
        update_bus_occupancy_stat();

        /* Masukkan penumpang tersebut ke dalam bus */
        list_file(LAST, LIST_BUS);
        bus.passenger_count++;

        if ((double) bus.passenger_count > stat_bus_occupancy.max) {
            stat_bus_occupancy.max = (double) bus.passenger_count;
        }

        total_loaded++;

        /* Bangkitkan random loading time independen per orang: Uniform(25, 35) */
        total_loading_time += uniform(LOAD_MIN, LOAD_MAX, STREAM_LOAD);
    }

    if (bus_stop_count <= MAX_STOPS_TO_PRINT) {
        if (m > 0) {
            printf("[t=%9.2f s]   -> Loading duration = %.2f s (Finish at t=%.2f)\n",
               sim_time, total_loading_time, sim_time + total_loading_time);
        } else {
            printf("[t=%9.2f s]   -> No passengers loaded (Duration = 0.00 s)\n", sim_time);
        }
    }

    /* Jadwalkan akhir proses loading */
    event_schedule(sim_time + total_loading_time, EVENT_END_LOADING);
}

/* ========================================================================== */
/* 7. BUS LOADING END & DEPARTURE HANDLERS                                    */
/* ========================================================================== */

void handle_end_loading(void)
{
    float min_departure_time;
    float departure_time;

    /* Evaluasi aturan minimal 5 menit (300 detik) di halte */
    min_departure_time = bus.arrival_time_at_stop + MIN_STOP_TIME;

    if (min_departure_time > sim_time) {
        departure_time = min_departure_time;
        if (bus_stop_count <= MAX_STOPS_TO_PRINT) {
            printf("[t=%9.2f s]   -> End Loading. Bus onboard = %d. Waiting until min stop time (t=%.2f s)\n",
                   sim_time, bus.passenger_count, departure_time);
        }
    } else {
        departure_time = sim_time;
        if (bus_stop_count <= MAX_STOPS_TO_PRINT) {
            printf("[t=%9.2f s]   -> End Loading. Bus onboard = %d. Service >= 300 s, departing immediately!\n",
                   sim_time, bus.passenger_count);
        }
    }

    /* Jadwalkan keberangkatan bus */
    event_schedule(departure_time, EVENT_BUS_DEPARTURE);
}

void handle_bus_departure(void)
{
    int curr = bus.location;
    int next;
    float travel_time;

    /* Catat statistik Stop Time di halte saat ini */
    double stop_duration = (double)(sim_time - bus.arrival_time_at_stop);
    record_discrete_stat(&stat_stop_time[curr], stop_duration);

    /* Tentukan rute berikutnya dan waktu tempuh: 3 -> 1 -> 2 -> 3 */
    if (curr == 3) {
        next = 1;
        travel_time = TRAVEL_TIME_3_1; /* 540 s */
    } else if (curr == 1) {
        next = 2;
        travel_time = TRAVEL_TIME_1_2; /* 120 s */
    } else { /* curr == 2 */
        next = 3;
        travel_time = TRAVEL_TIME_2_3; /* 540 s */
    }

    if (bus_stop_count <= MAX_STOPS_TO_PRINT) {
        printf("[t=%9.2f s] BUS DEPARTURE from Loc %d -> Loc %d | Stop Time: %.2f s | Onboard: %d/20 | Travel: %.1f s\n",
               sim_time, curr, next, stop_duration, bus.passenger_count, travel_time);
    }

    /* Update lokasi bus ke rute berikutnya */
    bus.location = next;

    /* Jadwalkan kedatangan di halte berikutnya */
    event_schedule(sim_time + travel_time, EVENT_BUS_ARRIVAL);
}

/* ========================================================================== */
/* 8. INITIALIZATION                                                          */
/* ========================================================================== */

void init_model(void)
{
    int i;

    init_simlib();

    /* Inisialisasi state bus awal */
    bus.location = 3;               /* Awalnya di Car Rental (Lokasi 3) */
    bus.passenger_count = 0;        /* Kosong */
    bus.arrival_time_at_stop = 0.0f;

    unloading_count = 0;
    next_passenger_id = 1;
    total_arrivals[1] = 0;
    total_arrivals[2] = 0;
    total_arrivals[3] = 0;
    total_unloaded = 0;
    total_loaded = 0;
    bus_stop_count = 0;

    last_arrival_at_loc3 = 0.0f;
    arrival_at_loc3_count = 0;

    /* Reset seluruh akumulator statistik */
    for (i = 1; i <= 3; i++) {
        reset_time_avg_stat(&stat_queue_len[i]);
        reset_discrete_stat(&stat_queue_delay[i]);
        reset_discrete_stat(&stat_stop_time[i]);
    }
    reset_time_avg_stat(&stat_bus_occupancy);
    reset_discrete_stat(&stat_loop_time);

    for (i = 0; i < 4; i++) {
        reset_discrete_stat(&stat_tis_route[i]);
    }
    reset_discrete_stat(&stat_tis_overall);

    printf("===================================================================\n");
    printf("   CAR-RENTAL SYSTEM SIMULATION (PROBLEM 2.38 - AVERILL M. LAW)    \n");
    printf("   TAHAP 9: IMPLEMENTASI STATISTIK SIMULASI LENGKAP               \n");
    printf("===================================================================\n");
    printf("Bus Capacity           : %d passengers\n", BUS_CAPACITY);
    printf("Unloading Time         : Uniform(%.1f, %.1f) s per passenger\n", UNLOAD_MIN, UNLOAD_MAX);
    printf("Loading Time           : Uniform(%.1f, %.1f) s per passenger\n", LOAD_MIN, LOAD_MAX);
    printf("Minimum Stop Time      : %.1f detik (5.0 menit)\n", MIN_STOP_TIME);
    printf("Target Simulation Time : %.1f detik (80.0 jam)\n", SIMULATION_TIME);
    printf("===================================================================\n\n");

    /* 1. Jadwalkan bus tiba di Lokasi 3 pada t = 0 */
    event_schedule(0.0f, EVENT_BUS_ARRIVAL);

    /* 2. Jadwalkan kedatangan pertama penumpang di masing-masing lokasi */
    event_schedule(expon(MEAN_ARRIV_1, STREAM_ARRIV_1), EVENT_ARRIVAL_1);
    event_schedule(expon(MEAN_ARRIV_2, STREAM_ARRIV_2), EVENT_ARRIVAL_2);
    event_schedule(expon(MEAN_ARRIV_3, STREAM_ARRIV_3), EVENT_ARRIVAL_3);

    /* 3. Jadwalkan akhir simulasi */
    event_schedule(SIMULATION_TIME, EVENT_END_SIMULATION);
}

/* ========================================================================== */
/* 9. PRINT FINAL STATISTICS REPORT & SANITY CHECKS                           */
/* ========================================================================== */

void print_statistics(void)
{
    int i;
    const char *loc_names[4] = {"", "Terminal 1", "Terminal 2", "Car Rental"};
    const char *route_names[4] = {
        "Location 1 -> 3 (Terminal 1 -> Car Rental)",
        "Location 2 -> 3 (Terminal 2 -> Car Rental)",
        "Location 3 -> 1 (Car Rental -> Terminal 1)",
        "Location 3 -> 2 (Car Rental -> Terminal 2)"
    };

    long total_arr = next_passenger_id - 1;
    long total_in_q = list_size[LIST_Q1] + list_size[LIST_Q2] + list_size[LIST_Q3];
    long completed_and_remaining = total_unloaded + total_in_q + bus.passenger_count;
    long diff = total_arr - completed_and_remaining;

    printf("\n");
    printf("===================================================================\n");
    printf("               LAPORAN RESMI HASIL SIMULASI FINAL                  \n");
    printf("            (Problem 2.38 - Simulation Modeling & Analysis)       \n");
    printf("===================================================================\n\n");

    /* ================================================================== */
    /* 1. SIMULATION SUMMARY                                              */
    /* ================================================================== */
    printf("===================================================================\n");
    printf("1. SIMULATION SUMMARY\n");
    printf("===================================================================\n");
    printf("  Simulation time             : %10.2f seconds (%.2f hours)\n", sim_time, sim_time / 3600.0);
    printf("  Total passenger arrivals    : %10ld passengers\n", total_arr);
    printf("    - Arrivals at Location 1  : %10ld passengers\n", total_arrivals[1]);
    printf("    - Arrivals at Location 2  : %10ld passengers\n", total_arrivals[2]);
    printf("    - Arrivals at Location 3  : %10ld passengers\n", total_arrivals[3]);
    printf("  Completed passengers        : %10ld passengers\n", total_unloaded);
    printf("  Passengers remaining in Q1  : %10d passengers\n", list_size[LIST_Q1]);
    printf("  Passengers remaining in Q2  : %10d passengers\n", list_size[LIST_Q2]);
    printf("  Passengers remaining in Q3  : %10d passengers\n", list_size[LIST_Q3]);
    printf("  Total remaining in queues   : %10ld passengers\n", total_in_q);
    printf("  Passengers remaining on bus : %10d passengers\n", bus.passenger_count);
    printf("  Total bus stops visited     : %10d stops\n", bus_stop_count);
    printf("  Total completed bus loops   : %10ld loops\n", stat_loop_time.count);
    printf("===================================================================\n\n");

    /* ================================================================== */
    /* 2. QUEUE LENGTH STATISTICS                                         */
    /* ================================================================== */
    printf("===================================================================\n");
    printf("2. QUEUE LENGTH STATISTICS (Time-Average & Maximum)\n");
    printf("===================================================================\n");
    for (i = 1; i <= 3; i++) {
        double avg_q = stat_queue_len[i].area / sim_time;
        printf("  Location %d (%-11s):\n", i, loc_names[i]);
        printf("    Time-average Queue Length : %8.4f passengers\n", avg_q);
        printf("    Maximum Queue Length      : %8.0f passengers\n", stat_queue_len[i].max);
        printf("    Final Queue Length        : %8d passengers\n", list_size[i]);
    }

    /* ================================================================== */
    /* 3. QUEUE DELAY STATISTICS                                          */
    /* ================================================================== */
    printf("\n===================================================================\n");
    printf("3. QUEUE DELAY STATISTICS (Minutes & Seconds)\n");
    printf("===================================================================\n");
    for (i = 1; i <= 3; i++) {
        double avg_d = get_discrete_avg(&stat_queue_delay[i]);
        printf("  Location %d (%-11s) [Observations: %ld]:\n", i, loc_names[i], stat_queue_delay[i].count);
        printf("    Average Queue Delay       : %8.2f s (%6.2f min)\n", avg_d, avg_d / 60.0);
        printf("    Maximum Queue Delay       : %8.2f s (%6.2f min)\n",
               stat_queue_delay[i].max, stat_queue_delay[i].max / 60.0);
    }

    /* ================================================================== */
    /* 4. BUS OCCUPANCY STATISTICS                                        */
    /* ================================================================== */
    printf("\n===================================================================\n");
    printf("4. BUS OCCUPANCY STATISTICS (Time-Average & Maximum)\n");
    printf("===================================================================\n");
    {
        double avg_occ = stat_bus_occupancy.area / sim_time;
        printf("  Time-average Bus Occupancy  : %8.4f passengers\n", avg_occ);
        printf("  Maximum Bus Occupancy       : %8.0f passengers (Capacity: %d)\n",
               stat_bus_occupancy.max, BUS_CAPACITY);
        printf("  Final Bus Occupancy         : %8d passengers\n", bus.passenger_count);
    }

    /* ================================================================== */
    /* 5. BUS STOP TIME STATISTICS                                        */
    /* ================================================================== */
    printf("\n===================================================================\n");
    printf("5. BUS STOP TIME STATISTICS (Minutes & Seconds)\n");
    printf("===================================================================\n");
    for (i = 1; i <= 3; i++) {
        double avg_stop = get_discrete_avg(&stat_stop_time[i]);
        double min_stop = (stat_stop_time[i].count > 0) ? stat_stop_time[i].min : 0.0;
        printf("  Location %d (%-11s) [Visits: %ld]:\n", i, loc_names[i], stat_stop_time[i].count);
        printf("    Minimum Stop Time         : %8.2f s (%6.2f min)\n", min_stop, min_stop / 60.0);
        printf("    Average Stop Time         : %8.2f s (%6.2f min)\n", avg_stop, avg_stop / 60.0);
        printf("    Maximum Stop Time         : %8.2f s (%6.2f min)\n",
               stat_stop_time[i].max, stat_stop_time[i].max / 60.0);
    }

    /* ================================================================== */
    /* 6. BUS LOOP TIME STATISTICS                                        */
    /* ================================================================== */
    printf("\n===================================================================\n");
    printf("6. BUS LOOP TIME STATISTICS (Location 3 -> Location 3)\n");
    printf("===================================================================\n");
    {
        double avg_loop = get_discrete_avg(&stat_loop_time);
        double min_loop = (stat_loop_time.count > 0) ? stat_loop_time.min : 0.0;
        printf("  Total Completed Loops       : %ld loops\n", stat_loop_time.count);
        printf("    Minimum Loop Time         : %8.2f s (%6.2f min)\n", min_loop, min_loop / 60.0);
        printf("    Average Loop Time         : %8.2f s (%6.2f min)\n", avg_loop, avg_loop / 60.0);
        printf("    Maximum Loop Time         : %8.2f s (%6.2f min)\n",
               stat_loop_time.max, stat_loop_time.max / 60.0);
    }

    /* ================================================================== */
    /* 7. PASSENGER TIME IN SYSTEM                                        */
    /* ================================================================== */
    printf("\n===================================================================\n");
    printf("7. PASSENGER TIME IN SYSTEM (By Route & Overall)\n");
    printf("===================================================================\n");
    {
        double avg_overall = get_discrete_avg(&stat_tis_overall);
        double min_overall = (stat_tis_overall.count > 0) ? stat_tis_overall.min : 0.0;
        printf("  OVERALL PASSENGERS [Total Delivered: %ld]:\n", stat_tis_overall.count);
        printf("    Minimum Time in System    : %8.2f s (%6.2f min)\n", min_overall, min_overall / 60.0);
        printf("    Average Time in System    : %8.2f s (%6.2f min)\n", avg_overall, avg_overall / 60.0);
        printf("    Maximum Time in System    : %8.2f s (%6.2f min)\n",
               stat_tis_overall.max, stat_tis_overall.max / 60.0);
    }
    printf("  -----------------------------------------------------------------\n");
    for (i = 0; i < 4; i++) {
        double avg_tis = get_discrete_avg(&stat_tis_route[i]);
        double min_tis = (stat_tis_route[i].count > 0) ? stat_tis_route[i].min : 0.0;
        printf("  Route %s [Delivered: %ld]:\n", route_names[i], stat_tis_route[i].count);
        printf("    Minimum Time in System    : %8.2f s (%6.2f min)\n", min_tis, min_tis / 60.0);
        printf("    Average Time in System    : %8.2f s (%6.2f min)\n", avg_tis, avg_tis / 60.0);
        printf("    Maximum Time in System    : %8.2f s (%6.2f min)\n",
               stat_tis_route[i].max, stat_tis_route[i].max / 60.0);
    }

    /* ================================================================== */
    /* 8. SANITY CHECKS                                                   */
    /* ================================================================== */
    printf("\n===================================================================\n");
    printf("8. SANITY CHECKS\n");
    printf("===================================================================\n");
    int pass_all = 1;

    /* Check A: Simulation time = 288000 */
    int sc1 = (sim_time == (float)SIMULATION_TIME);
    printf("  Simulation time = 288000 s           : %s (Actual: %.2f s)\n", sc1 ? "PASS" : "FAIL", sim_time);
    if (!sc1) pass_all = 0;

    /* Check B: Maximum occupancy <= 20 */
    int sc2 = (stat_bus_occupancy.max <= (double)BUS_CAPACITY);
    printf("  Maximum occupancy <= 20              : %s (Actual: %.0f passengers)\n", sc2 ? "PASS" : "FAIL", stat_bus_occupancy.max);
    if (!sc2) pass_all = 0;

    /* Check C: Minimum stop time >= 300 s */
    int sc3 = (stat_stop_time[1].min >= MIN_STOP_TIME &&
               stat_stop_time[2].min >= MIN_STOP_TIME &&
               stat_stop_time[3].min >= MIN_STOP_TIME);
    printf("  Minimum stop time >= 300 s           : %s (L1=%.2f s, L2=%.2f s, L3=%.2f s)\n",
           sc3 ? "PASS" : "FAIL", stat_stop_time[1].min, stat_stop_time[2].min, stat_stop_time[3].min);
    if (!sc3) pass_all = 0;

    /* Check D: Queue delay >= 0 */
    int sc4 = (stat_queue_delay[1].min >= 0.0 &&
               stat_queue_delay[2].min >= 0.0 &&
               stat_queue_delay[3].min >= 0.0);
    printf("  Queue delay >= 0                     : %s (Min L1=%.2f, L2=%.2f, L3=%.2f)\n",
           sc4 ? "PASS" : "FAIL", stat_queue_delay[1].min, stat_queue_delay[2].min, stat_queue_delay[3].min);
    if (!sc4) pass_all = 0;

    /* Check E: Time in system >= 0 */
    int sc5 = (stat_tis_overall.min >= 0.0);
    printf("  Time in system >= 0                  : %s (Min TIS: %.2f s)\n", sc5 ? "PASS" : "FAIL", stat_tis_overall.min);
    if (!sc5) pass_all = 0;

    /* Check F: Loop time >= 2100 s */
    int sc6 = (stat_loop_time.min >= 2100.0);
    printf("  Loop time >= 2100 s                  : %s (Min Loop: %.2f s)\n", sc6 ? "PASS" : "FAIL", stat_loop_time.min);
    if (!sc6) pass_all = 0;

    printf("  -----------------------------------------------------------------\n");
    printf("  Sanity Checks Overall Status         : %s\n", pass_all ? "ALL PASS" : "FAIL");

    /* ================================================================== */
    /* 9. CONSERVATION CHECK                                              */
    /* ================================================================== */
    printf("\n===================================================================\n");
    printf("9. CONSERVATION CHECK\n");
    printf("===================================================================\n");
    printf("  Total arrivals                       : %ld passengers\n", total_arr);
    printf("  Completed passengers                 : %ld passengers\n", total_unloaded);
    printf("  Passengers in queues (Q1+Q2+Q3)      : %ld passengers (%d + %d + %d)\n",
           total_in_q, list_size[LIST_Q1], list_size[LIST_Q2], list_size[LIST_Q3]);
    printf("  Passengers on bus                    : %d passengers\n", bus.passenger_count);
    printf("  Sum (Completed + Queues + Bus)       : %ld passengers\n", completed_and_remaining);
    printf("  Difference (Arrivals - Sum)          : %ld\n", diff);
    printf("  Conservation Status                  : %s\n", (diff == 0) ? "PASS (EXACT MATCH)" : "FAIL");
    printf("===================================================================\n\n");
}

/* ========================================================================== */
/* 10. MAIN SIMULATION LOOP                                                   */
/* ========================================================================== */

int main(void)
{
    int simulation_running = 1;
    int i;

    init_model();

    while (simulation_running) {

        timing();

        switch (next_event_type) {

            case EVENT_ARRIVAL_1:
                handle_passenger_arrival(1);
                break;

            case EVENT_ARRIVAL_2:
                handle_passenger_arrival(2);
                break;

            case EVENT_ARRIVAL_3:
                handle_passenger_arrival(3);
                break;

            case EVENT_BUS_ARRIVAL:
                handle_bus_arrival();
                break;

            case EVENT_END_UNLOADING:
                handle_end_unloading();
                break;

            case EVENT_END_LOADING:
                handle_end_loading();
                break;

            case EVENT_BUS_DEPARTURE:
                handle_bus_departure();
                break;

            case EVENT_END_SIMULATION:
                printf("\n[t=%9.2f s] EVENT_END_SIMULATION tercapai! Menghentikan event loop.\n", sim_time);
                simulation_running = 0;
                break;

            default:
                break;
        }
    }

    /* Final Update untuk Statistik Time-Average pada t = 288000 s */
    for (i = 1; i <= 3; i++) {
        update_queue_stat(i);
    }
    update_bus_occupancy_stat();

    /* Tampilkan Laporan Statistik Lengkap */
    print_statistics();

    return 0;
}
