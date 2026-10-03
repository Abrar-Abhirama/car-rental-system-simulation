/* simlib.c
   Implementation of SIMLIB (C version)
   Based on Simulation Modeling and Analysis by Averill M. Law & W. David Kelton
*/

#include "simlib.h"

/* Global variables */
int   next_event_type = 0;
float sim_time = 0.0f;
float transfer[MAX_ATTR + 1];
int   list_size[MAX_LIST + 1];

/* Internal node structure for dynamic doubly-linked lists */
typedef struct node {
    float attr[MAX_ATTR + 1];
    struct node *pred;
    struct node *succ;
} Node;

typedef struct {
    Node *head;
    Node *tail;
    int   size;
} ListHeader;

static ListHeader lists[MAX_LIST + 1];

/* Random number generator seeds array (streams 1 to 100) */
static long zrng[101] = {
             0,
    1973272912,  281629770,   20006270, 1280689831, 2096730327,  654572301,
     124838639,  241124623, 1998509740, 1909994759,  935661438,  826066224,
    1667493635, 1215437877, 1874221782, 1373578768,  538548981, 1073841571,
     374092498, 1699946467, 1375990263,  535492212, 1709424756, 1279184562,
    1098656111, 1421035548, 1506509930,  990710606, 1870628286,  596950346,
    1484085448, 1146747593,  252554746, 1729007621, 1269550519, 1147573038,
     842525166, 1879791883,  870878235,  275727976, 1024300645, 1823908865,
     846205837, 1092809633,  454940569,  927237889, 1239965384,  368413159,
    1996957798, 1438515324, 1836109968, 1819777174, 1813636417,  597799517,
     617833502, 1221742468, 1489065112, 1704257143, 1017409559, 1359196557,
     242130794,  556100140,  914047647, 1071221992,  948950003, 1965906353,
     758778835,  188151240, 1599818833,  643444031,  686851508, 1198642735,
    1443657788, 1774351613,  945532557, 1386766487, 1007802871, 1373574512,
     821434971,  982117565, 1289196881, 1290333202, 1113271798,  826066224,
     518625983, 1928374650, 1122334455,  667788990, 1472583690,  369258147,
     789123456,  987654321, 1234567890, 1987654321,  112233445,  556677889,
     998877665,  443322110,  135792468,  246813579
};

/* LCG Constants (Law & Kelton standard) */
#define MODLUS 2147483647L
#define MULT1  24112L
#define MULT2  26143L

/* Statistical accumulators storage */
#define MAX_SVAR 25
#define MAX_TVAR 25

static struct {
    int   count;
    float sum;
    float max;
    float min;
} s_vars[MAX_SVAR + 1];

static struct {
    float last_time;
    float last_val;
    float area;
    float max;
    float min;
} t_vars[MAX_TVAR + 1];

/* -------------------------------------------------------------------------- */
/* Initialization */
/* -------------------------------------------------------------------------- */
void init_simlib(void)
{
    int i;
    sim_time = 0.0f;
    next_event_type = 0;

    for (i = 0; i <= MAX_ATTR; ++i) {
        transfer[i] = 0.0f;
    }

    for (i = 1; i <= MAX_LIST; ++i) {
        /* Free any existing nodes */
        Node *curr = lists[i].head;
        while (curr != NULL) {
            Node *tmp = curr;
            curr = curr->succ;
            free(tmp);
        }
        lists[i].head = NULL;
        lists[i].tail = NULL;
        lists[i].size = 0;
        list_size[i]  = 0;
    }

    for (i = 1; i <= MAX_SVAR; ++i) {
        s_vars[i].count = 0;
        s_vars[i].sum   = 0.0f;
        s_vars[i].max   = -1.0e30f;
        s_vars[i].min   =  1.0e30f;
    }

    for (i = 1; i <= MAX_TVAR; ++i) {
        t_vars[i].last_time = 0.0f;
        t_vars[i].last_val  = 0.0f;
        t_vars[i].area      = 0.0f;
        t_vars[i].max       = -1.0e30f;
        t_vars[i].min       =  1.0e30f;
    }
}

/* -------------------------------------------------------------------------- */
/* Generalized List Management */
/* -------------------------------------------------------------------------- */
void list_file(int option, int list)
{
    int a;
    Node *new_node;

    if (list < 1 || list > MAX_LIST) {
        fprintf(stderr, "simlib: Invalid list number %d in list_file\n", list);
        exit(1);
    }

    new_node = (Node *) malloc(sizeof(Node));
    if (!new_node) {
        fprintf(stderr, "simlib: Out of memory in list_file\n");
        exit(1);
    }

    for (a = 1; a <= MAX_ATTR; ++a) {
        new_node->attr[a] = transfer[a];
    }
    new_node->pred = NULL;
    new_node->succ = NULL;

    if (lists[list].size == 0) {
        /* List is currently empty */
        lists[list].head = new_node;
        lists[list].tail = new_node;
    } else if (option == FIRST) {
        new_node->succ = lists[list].head;
        lists[list].head->pred = new_node;
        lists[list].head = new_node;
    } else if (option == LAST) {
        new_node->pred = lists[list].tail;
        lists[list].tail->succ = new_node;
        lists[list].tail = new_node;
    } else if (option == INCREASING) {
        /* Ordered by attribute 1 ascending (used for event list) */
        Node *curr = lists[list].head;
        while (curr != NULL && curr->attr[1] <= new_node->attr[1]) {
            curr = curr->succ;
        }
        if (curr == lists[list].head) {
            new_node->succ = lists[list].head;
            lists[list].head->pred = new_node;
            lists[list].head = new_node;
        } else if (curr == NULL) {
            new_node->pred = lists[list].tail;
            lists[list].tail->succ = new_node;
            lists[list].tail = new_node;
        } else {
            new_node->pred = curr->pred;
            new_node->succ = curr;
            curr->pred->succ = new_node;
            curr->pred = new_node;
        }
    } else if (option == DECREASING) {
        /* Ordered by attribute 1 descending */
        Node *curr = lists[list].head;
        while (curr != NULL && curr->attr[1] >= new_node->attr[1]) {
            curr = curr->succ;
        }
        if (curr == lists[list].head) {
            new_node->succ = lists[list].head;
            lists[list].head->pred = new_node;
            lists[list].head = new_node;
        } else if (curr == NULL) {
            new_node->pred = lists[list].tail;
            lists[list].tail->succ = new_node;
            lists[list].tail = new_node;
        } else {
            new_node->pred = curr->pred;
            new_node->succ = curr;
            curr->pred->succ = new_node;
            curr->pred = new_node;
        }
    }

    lists[list].size++;
    list_size[list] = lists[list].size;
}

void list_remove(int option, int list)
{
    int a;
    Node *rem_node;

    if (list < 1 || list > MAX_LIST) {
        fprintf(stderr, "simlib: Invalid list number %d in list_remove\n", list);
        exit(1);
    }
    if (lists[list].size == 0) {
        fprintf(stderr, "simlib: Underflow in list_remove on empty list %d\n", list);
        exit(1);
    }

    if (option == FIRST) {
        rem_node = lists[list].head;
        lists[list].head = rem_node->succ;
        if (lists[list].head != NULL) {
            lists[list].head->pred = NULL;
        } else {
            lists[list].tail = NULL;
        }
    } else { /* LAST */
        rem_node = lists[list].tail;
        lists[list].tail = rem_node->pred;
        if (lists[list].tail != NULL) {
            lists[list].tail->succ = NULL;
        } else {
            lists[list].head = NULL;
        }
    }

    for (a = 1; a <= MAX_ATTR; ++a) {
        transfer[a] = rem_node->attr[a];
    }

    free(rem_node);
    lists[list].size--;
    list_size[list] = lists[list].size;
}

/* -------------------------------------------------------------------------- */
/* Timing and Event Scheduling */
/* -------------------------------------------------------------------------- */
void timing(void)
{
    if (lists[LIST_EVENT].size == 0) {
        fprintf(stderr, "simlib: Event list empty at time %f\n", sim_time);
        exit(1);
    }

    list_remove(FIRST, LIST_EVENT);

    if (transfer[EVENT_TIME] < sim_time) {
        fprintf(stderr, "simlib: Attempted to advance backward in time: curr=%f, event=%f\n",
                sim_time, transfer[EVENT_TIME]);
        exit(1);
    }

    sim_time = transfer[EVENT_TIME];
    next_event_type = (int) transfer[EVENT_TYPE];
}

void event_schedule(float time_value, int event_type)
{
    float save_transfer[MAX_ATTR + 1];
    int a;

    /* Save caller's transfer buffer */
    for (a = 1; a <= MAX_ATTR; ++a) {
        save_transfer[a] = transfer[a];
    }

    transfer[EVENT_TIME] = time_value;
    transfer[EVENT_TYPE] = (float) event_type;

    list_file(INCREASING, LIST_EVENT);

    /* Restore caller's transfer buffer */
    for (a = 1; a <= MAX_ATTR; ++a) {
        transfer[a] = save_transfer[a];
    }
}

void event_cancel(int event_type)
{
    Node *curr = lists[LIST_EVENT].head;
    while (curr != NULL) {
        if ((int) curr->attr[EVENT_TYPE] == event_type) {
            /* Remove this node */
            if (curr->pred != NULL) {
                curr->pred->succ = curr->succ;
            } else {
                lists[LIST_EVENT].head = curr->succ;
            }
            if (curr->succ != NULL) {
                curr->succ->pred = curr->pred;
            } else {
                lists[LIST_EVENT].tail = curr->pred;
            }
            free(curr);
            lists[LIST_EVENT].size--;
            list_size[LIST_EVENT] = lists[LIST_EVENT].size;
            return;
        }
        curr = curr->succ;
    }
}

/* -------------------------------------------------------------------------- */
/* Random Variate Generation */
/* -------------------------------------------------------------------------- */
float lcgrand(int stream)
{
    long zi, lowprd, hi31;

    if (stream < 1 || stream > 100) {
        fprintf(stderr, "simlib: Invalid random stream %d in lcgrand\n", stream);
        exit(1);
    }

    zi     = zrng[stream];
    lowprd = (zi & 65535) * MULT1;
    hi31   = (zi >> 16) * MULT1 + (lowprd >> 16);
    zi     = ((lowprd & 65535) - MODLUS) +
             ((hi31 & 32767) << 16) + (hi31 >> 15);
    if (zi < 0) zi += MODLUS;

    lowprd = (zi & 65535) * MULT2;
    hi31   = (zi >> 16) * MULT2 + (lowprd >> 16);
    zi     = ((lowprd & 65535) - MODLUS) +
             ((hi31 & 32767) << 16) + (hi31 >> 15);
    if (zi < 0) zi += MODLUS;

    zrng[stream] = zi;
    return ((float) (zi >> 7)) / 16777216.0f;
}

void lcgrandst(long zset, int stream)
{
    if (stream >= 1 && stream <= 100) {
        zrng[stream] = zset;
    }
}

long lcgrandgt(int stream)
{
    if (stream >= 1 && stream <= 100) {
        return zrng[stream];
    }
    return 0;
}

float expon(float mean, int stream)
{
    return -mean * (float) log((double) lcgrand(stream));
}

float uniform(float a, float b, int stream)
{
    return a + lcgrand(stream) * (b - a);
}

/* -------------------------------------------------------------------------- */
/* Statistical Functions */
/* -------------------------------------------------------------------------- */
void sampst(float value, int var)
{
    if (var < 1 || var > MAX_SVAR) return;
    s_vars[var].count++;
    s_vars[var].sum += value;
    if (value > s_vars[var].max) s_vars[var].max = value;
    if (value < s_vars[var].min) s_vars[var].min = value;
}

void timest(float value, int var)
{
    if (var < 1 || var > MAX_TVAR) return;
    t_vars[var].area += (sim_time - t_vars[var].last_time) * t_vars[var].last_val;
    t_vars[var].last_time = sim_time;
    t_vars[var].last_val  = value;
    if (value > t_vars[var].max) t_vars[var].max = value;
    if (value < t_vars[var].min) t_vars[var].min = value;
}

void filest(int list)
{
    if (list < 1 || list > MAX_LIST) return;
    transfer[1] = (float) lists[list].size;
    transfer[2] = (float) lists[list].size;
}

void out_sampst(FILE *unit, int low_var, int high_var)
{
    int i;
    fprintf(unit, "\n Discrete Sample Statistics:\n");
    for (i = low_var; i <= high_var; ++i) {
        float avg = (s_vars[i].count > 0) ? s_vars[i].sum / s_vars[i].count : 0.0f;
        fprintf(unit, " Var %2d: Count=%6d  Avg=%12.4f  Min=%12.4f  Max=%12.4f\n",
                i, s_vars[i].count, avg, s_vars[i].min, s_vars[i].max);
    }
}

void out_timest(FILE *unit, int low_var, int high_var)
{
    int i;
    fprintf(unit, "\n Continuous Time-Average Statistics:\n");
    for (i = low_var; i <= high_var; ++i) {
        float total_area = t_vars[i].area + (sim_time - t_vars[i].last_time) * t_vars[i].last_val;
        float avg = (sim_time > 0.0f) ? total_area / sim_time : 0.0f;
        fprintf(unit, " Var %2d: Time-Avg=%12.4f  Min=%12.4f  Max=%12.4f\n",
                i, avg, t_vars[i].min, t_vars[i].max);
    }
}

void out_filest(FILE *unit, int low_list, int high_list)
{
    int i;
    fprintf(unit, "\n List Sizes:\n");
    for (i = low_list; i <= high_list; ++i) {
        fprintf(unit, " List %2d: Current Size=%4d\n", i, lists[i].size);
    }
}
