#define SPXE_APPLICATION
#include <spxe.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define WIDTH 800
#define HEIGHT 600
#define SUBSCALE 2
#define SUBSTEPS 1
#define DUST_POOL_SIZE 100
#define G 50.2F
#define MASS_MIN 1.0F
#define MASS_MAX 1.0F

typedef struct vec2 {
    float x, y;
} vec2;

struct dust {
    vec2 positions[DUST_POOL_SIZE];
    float timers[DUST_POOL_SIZE];
    int marker;
};

struct universe {
    size_t count;
    vec2* positions;
    vec2* velocities;
    struct dust* dusts;
    float* radiuses;
    float* sqradiuses;
    Px* colors;
};

static float frand(void)
{
    return (float)rand() / (float)RAND_MAX;
}

static float clampf(const float n, const float min, const float max)
{
    return n > max ? max : n < min ? min : n;
}

static float lerpf(const float a, const float b, const float t)
{
    return a + t * (b - a);
}

static Px pxlerp(const Px c1, const Px c2, float t)
{
    return (Px) {
        (uint8_t)lerpf((float)c1.r, (float)c2.r, t),
        (uint8_t)lerpf((float)c1.g, (float)c2.g, t),
        (uint8_t)lerpf((float)c1.b, (float)c2.b, t),
        255
    };
}

static vec2 mouse_position(void)
{
    int x, y;
    spxeMousePos(&x, &y);
    return (vec2){(float)x, (float)y};
}

static vec2 vec2_delta(vec2 p, vec2 v, float dT)
{
    return (vec2){p.x + v.x * dT, p.y + v.y * dT};
}

static vec2 vec2_norm(vec2 p)
{
    float d = sqrtf(p.x * p.x + p.y * p.y);
    return (vec2){p.x / d, p.y / d};
}

static int circle_point_overlap(const vec2 p, const vec2 q, const float sqr)
{
    const vec2 v = {p.x - q.x, p.y - q.y};
    return v.x * v.x + v.y * v.y <= sqr;
}

static int circle_overlap(const vec2 p1, const vec2 p2, const float r1, const float r2)
{
    const float r = r1 + r2;
    const float dx = p1.x - p2.x;
    const float dy = p1.y - p2.y;
    return dx * dx + dy * dy <= r * r;
}

static int universe_spawn_collision(const struct universe* universe, const size_t index)
{
    const float r = universe->radiuses[index];
    const vec2 p = universe->positions[index];
    for (size_t i = 0; i < index; ++i) {
        if (circle_overlap(p, universe->positions[i], r, universe->radiuses[i])) {
            return 1;
        }
    }
    return 0;
}

static void universe_spawn(struct universe* universe)
{
    const size_t count = universe->count;
    int width = spxe.scrres.width, height = spxe.scrres.height;
    const float scale = (float)(width < height ? width : height) / 30.0F;
    const float minr = scale * MASS_MIN, maxr = scale * MASS_MAX;
    memset(universe->velocities, 0, sizeof(vec2) * universe->count);
    for (size_t i = 0; i < count; ++i) {
        float n = 1.0F;
        universe->positions[i].x = frand() * (float)width;
        universe->positions[i].y = frand() * (float)height;
        universe->radiuses[i] = minr + frand() * maxr;
        while (universe_spawn_collision(universe, i)) {
            float m = n * 2.0F;
            universe->positions[i].x = (m * frand() - n + 0.5F) * (float)width;
            universe->positions[i].y = (m * frand() - n + 0.5F) * (float)height;
            universe->radiuses[i] = minr + frand() * maxr;
            n = m;
        }

        universe->sqradiuses[i] = universe->radiuses[i] * universe->radiuses[i];
        universe->colors[i].r = rand() % 256;
        universe->colors[i].g = rand() % 256;
        universe->colors[i].b = rand() % 256;
        universe->colors[i].a = 255;
        memset(universe->dusts + i, 0, sizeof(struct dust));
    }
}

static struct universe universe_create(const size_t count)
{
    struct universe universe;
    universe.count = count;
    universe.positions = malloc(count * sizeof(vec2));
    universe.velocities = calloc(count, sizeof(vec2));
    universe.radiuses = malloc(count * sizeof(float));
    universe.sqradiuses = malloc(count * sizeof(float));
    universe.colors = malloc(count * sizeof(Px));
    universe.dusts = calloc(count, sizeof(struct dust));
    universe_spawn(&universe);
    return universe;
}

static void universe_free(struct universe* universe)
{
    if (universe->count) {
        free(universe->positions);
        free(universe->velocities);
        free(universe->radiuses);
        free(universe->sqradiuses);
        free(universe->colors);
        free(universe->dusts);
        memset(universe, 0, sizeof(struct universe));
    }
}

static void universe_apply_gravity(const struct universe* universe)
{
    for (size_t i = 0; i < universe->count; ++i) {
        for (size_t j = 0; j < universe->count; ++j) {
            if (i != j) {
                const float dx = universe->positions[i].x - universe->positions[j].x;
                const float dy = universe->positions[i].y - universe->positions[j].y;
                const float sqr = dx * dx + dy * dy;
                const float f = G * universe->sqradiuses[j] / sqr;
                const float r = sqrtf(sqr);
                universe->velocities[i].x -= f * dx / r;
                universe->velocities[i].y -= f * dy / r;
            }
        }
    }
}

static void universe_collision_check(
    const struct universe* universe, const float dT, const size_t i)
{
    const float R = universe->radiuses[i];
    const vec2 P = vec2_delta(universe->positions[i], universe->velocities[i], dT);
    for (size_t j = 0; j < universe->count; ++j) {
        if (i == j) {
            continue;
        }

        vec2 q = vec2_delta(universe->positions[j], universe->velocities[j], dT);
        if (!circle_overlap(P, q, R, universe->radiuses[j])) {
            continue;
        }
        
        const float mass = universe->sqradiuses[i] + universe->sqradiuses[j];
        const float dx = q.x - P.x;
        const float dy = q.y - P.y;
        const float d = sqrtf(dx * dx + dy * dy);
        const vec2 n = {dx / d, dy / d};
        const float p = (universe->velocities[i].x * n.x + 
            universe->velocities[i].y * n.y - universe->velocities[j].x * n.x
            - universe->velocities[j].y * n.y) * 2.0F / mass;

        universe->velocities[i].x -= p * universe->sqradiuses[j] * n.x;
        universe->velocities[i].y -= p * universe->sqradiuses[j] * n.y;
        universe->velocities[j].x += p * universe->sqradiuses[i] * n.x;
        universe->velocities[j].y += p * universe->sqradiuses[i] * n.y;

        vec2 c1 = vec2_delta(universe->positions[i], universe->velocities[i], dT); 
        vec2 c2 = vec2_delta(universe->positions[j], universe->velocities[j], dT); 

        if (circle_overlap(c1, c2, universe->radiuses[i], universe->radiuses[j])) {
            float r = R + universe->radiuses[j];
            float k1 = R / r;
            float k2 = 1.0F - k1;
            vec2 d = {c2.x - c1.x, c2.y - c1.y};
            vec2 n = vec2_norm(d);
            vec2 v = {d.x - n.x * r, d.y - n.y * r};
            universe->velocities[i].x = k1 * v.x / dT;
            universe->velocities[i].y = k1 * v.y / dT;
            universe->velocities[j].x = -k2 * v.x / dT;
            universe->velocities[j].y = -k2 * v.y / dT;
        } else {
            universe_collision_check(universe, dT, j);
            universe_collision_check(universe, dT, i);
        }   
        break;
    }
}

static void universe_update(
    const struct universe* universe, const int hover, const float dT)
{
    const size_t h = (size_t)hover;
    for (size_t i = 0; i < universe->count; ++i) {
        if (h != i) {
            universe_collision_check(universe, dT, i);
        }
    }

    for (size_t i = 0; i < universe->count; ++i) { 
        if (h != i) {
            universe->positions[i].x += universe->velocities[i].x * dT;
            universe->positions[i].y += universe->velocities[i].y * dT;
        }
    }
}

static void circle_render_subsample(
    Px* pixbuf, const vec2 p, const float r, const float sqr, const Px c)
{
    const int width = spxe.scrres.width, height = spxe.scrres.height;
    const int startx = clampf(p.x - r, 0.0F, (float)(width - 1));
    const int starty = clampf(p.y - r, 0.0F, (float)(height - 1));
    const int endx = clampf(p.x + r + 1.0F, 0.0F, (float)(width - 1));
    const int endy = clampf(p.y + r + 1.0F, 0.0F, (float)(height - 1));
    for (int y = starty; y <= endy; ++y) {
        const float dy = p.y - (float)y;
        for (int x = startx; x <= endx; ++x) {
            int count = 0;
            const float dx = p.x - (float)x;
            for (int sy = 0; sy < SUBSCALE; ++sy) {
                const float sdy = dy + (float)sy / (float)SUBSCALE;
                const float sqdy = sdy * sdy;
                for (int sx = 0; sx < SUBSCALE; ++sx) {
                    const float sdx = dx + (float)sx / (float)SUBSCALE;
                    count += sdx * sdx + sqdy <= sqr;
                }
            }
            Px* px = pixbuf + y * width + x;
            const Px col = pxlerp(*px, c, (float)count / (float)(SUBSCALE * SUBSCALE));
            *px = col;
        }
    }
}

static void circle_render_subsample_traced(
    Px* pixbuf, const vec2 p, const float r, const float sqr, const Px c, const float t)
{
    const int width = spxe.scrres.width, height = spxe.scrres.height;
    const int startx = clampf(p.x - r, 0.0F, (float)(width - 1));
    const int starty = clampf(p.y - r, 0.0F, (float)(height - 1));
    const int endx = clampf(p.x + r + 1.0F, 0.0F, (float)(width - 1));
    const int endy = clampf(p.y + r + 1.0F, 0.0F, (float)(height - 1));
    for (int y = starty; y <= endy; ++y) {
        const float dy = p.y - (float)y;
        for (int x = startx; x <= endx; ++x) {
            int count = 0;
            const float dx = p.x - (float)x;
            for (int sy = 0; sy < SUBSCALE; ++sy) {
                const float sdy = dy + (float)sy / (float)SUBSCALE;
                const float sqdy = sdy * sdy;
                for (int sx = 0; sx < SUBSCALE; ++sx) {
                    const float sdx = dx + (float)sx / (float)SUBSCALE;
                    const float sd = sdx * sdx + sqdy;
                    count += (sd <= sqr) && (sd >= t);
                }
            }
            Px* px = pixbuf + y * width + x;
            const Px col = pxlerp(*px, c, (float)count / (float)(SUBSCALE * SUBSCALE));
            *px = col;
        }
    }
}

static void universe_dust_render(
    const struct universe* universe, Px* pixbuf, const vec2 cam, const float scale)
{
    const int w = spxe.scrres.width, h = spxe.scrres.height;
    const vec2 hres = {(float)w * 0.5F, (float)h * 0.5F};
    for (size_t i = 0; i < universe->count; ++i) { 
        for (size_t j = 0; j < DUST_POOL_SIZE; ++j) {
            if (universe->dusts[i].timers[j] > 0.0F) {
                const vec2 dp = {
                    universe->dusts[i].positions[j].x - cam.x,
                    universe->dusts[i].positions[j].y - cam.y
                };
                const vec2 dq = {
                    hres.x + (hres.x - dp.x) * scale, 
                    hres.y + (hres.y - dp.y) * scale
                };
                const int x = (int)dq.x;
                const int y = (int)dq.y;
                if (x >= 0 && y >= 0 && x < w && y < h) {
                    pixbuf[y * w + x] = pxlerp(
                        pixbuf[y * w + x],
                        universe->colors[i],
                        universe->dusts[i].timers[j]
                    );
                }
                universe->dusts[i].timers[j] -= 1.0F / (float)DUST_POOL_SIZE;
            }
        }
        universe->dusts[i].positions[universe->dusts[i].marker] = universe->positions[i];
        universe->dusts[i].timers[universe->dusts[i].marker] = 1.0F;
        universe->dusts[i].marker = (universe->dusts[i].marker + 1) % DUST_POOL_SIZE;
    }
}

static int universe_render(
    const struct universe* universe,    Px* pixbuf,             const vec2 cam, 
    const vec2 mouse,                   const float scale)
{
    int mouse_overlap = -1;
    const int w = spxe.scrres.width, h = spxe.scrres.height;
    const vec2 hres = {(float)w * 0.5F, (float)h * 0.5F};
    for (size_t i = 0; i < universe->count; ++i) { 
        float r = universe->radiuses[i] * scale;
        float sqr = r * r;
        vec2 p = {universe->positions[i].x - cam.x, universe->positions[i].y - cam.y};
        vec2 q = {hres.x + (hres.x - p.x) * scale, hres.y + (hres.y - p.y) * scale};
        if (mouse_overlap == -1 && circle_point_overlap(q, mouse, sqr)) {
            float t = r - 2.0F;
            Px n = {
                255 - universe->colors[i].r, 
                255 - universe->colors[i].g,
                255 - universe->colors[i].b, 
                255
            };
            circle_render_subsample_traced(pixbuf, q, r, sqr, n, t * t);
            mouse_overlap = i;
        } else {
            circle_render_subsample(pixbuf, q, r, sqr,  universe->colors[i]);
        }
    }
    return mouse_overlap;
}

static Px* universe_background_create(const int total, const int prob)
{
    const Px black = {0, 0, 0, 255};
    const Px purple = {75, 0, 75, 255};
    const Px white = {255, 255, 255, 255};
    const int width = spxe.scrres.width, height = spxe.scrres.height;
    Px* bg = malloc(width * height * sizeof(Px));
    for (int y = 0; y < height; ++y) {
        const Px c = pxlerp(purple, black, (float)y / (float)height);
        const int yw = width * y;
        for (int x = 0; x < width; ++x) {
            if (rand() % total < prob) {
                bg[yw + x] = white;
            } else {
                bg[yw + x] = c;
            }
        }
    }
    return bg;
}

static int gsim_error(const char* fmt, const char* exe, const char* arg)
{
    fprintf(stderr, "%s: ", exe);
    fprintf(stderr, fmt, arg);
    return EXIT_FAILURE;
}

int main(const int argc, const char** argv)
{
    Px* pixbuf, *sky;
    struct universe universe;
    int i, count, width = WIDTH, height = HEIGHT;
    int gravity = 1, paused = 0, hover = -1, trail = 1;
    float t, T, dT, dK = 1.0F, scale = 1.0F;
    vec2 cam = {0.0F, 0.0F}, mouse = {0.0F, 0.0F};
    
    srand(time(NULL));
    count = rand() % 10 + 5;
    
    for (i = 1; i < argc; ++i) {
        int *var = NULL;
        if (!strcmp(argv[i], "-w")) {
            var = &width;
        } else if (!strcmp(argv[i], "-h")) {
            var = &height;
        } else if (argv[i][0] >= '0' && argv[i][0] <= '9') {
            count = atoi(argv[i]);
        } else {
            return gsim_error("illegal option '%s'\n", argv[0], argv[i]);
        }

        if (var) {
            if (i + 1 == argc) {
                return gsim_error(
                    "expected numeric argument afte '%s' option\n",
                    argv[0],
                    argv[i]
                );
            }
            
            *var = atoi(argv[i + 1]);
            if (*var < 1) {
                return gsim_error(
                    "argument for option '%s' must be a non-zero unsigned integer\n",
                    argv[0],
                    argv[i]
                );
            }
            ++i;
        }
    }

    pixbuf = spxeStart("bodies", WIDTH, HEIGHT, width, height);
    universe = universe_create(count);
    sky = universe_background_create(1000, 2);

    t = spxeTime();
    while (spxeRun(pixbuf)) {
        T = spxeTime();
        dT = T - t;
        t = T;

        const float dM = dT * 100.0F;
        const vec2 m = mouse_position();
        const int clicked = spxeMouseDown(LEFT);

        if (spxeKeyPressed(ESCAPE)) {
            break;
        }
        if (spxeKeyPressed(R)) {
            cam.x = 0.0F;
            cam.y = 0.0F;
            scale = 1.0F;
            universe_spawn(&universe);
        }
        if (spxeKeyPressed(G)) {
            gravity = !gravity;
        }
        if (spxeKeyPressed(SPACE)) {
            paused = !paused;
        }
        if (spxeKeyPressed(LEFT_SHIFT)) {
            trail = !trail;
        }

        if (clicked) {
            vec2 d = {m.x - mouse.x, m.y - mouse.y};
            if (hover != -1) {
                vec2 p = universe.positions[hover];
                vec2 q = {p.x - d.x / scale, p.y - d.y / scale};
                universe.velocities[hover].x = (q.x - p.x) / (dT * dK);
                universe.velocities[hover].y = (q.y - p.y) / (dT * dK);
                universe.positions[hover] = q;
            } else {
                cam.x += d.x / scale;
                cam.y += d.y / scale;
            }
        }

        if (spxeKeyDown(W)) {
            cam.y -= dM / scale;
        }
        if (spxeKeyDown(S)) {
            cam.y += dM/ scale;
        }
        if (spxeKeyDown(D)) {
            cam.x -= dM / scale;
        }
        if (spxeKeyDown(A)) {
            cam.x += dM / scale;
        }
        if (spxeKeyDown(Z)) {
            scale += dT * scale;
        }
        if (spxeKeyDown(X)) {
            scale -= dT * scale;
        }
        if (spxeKeyDown(P)) {
            dK += dT;
        }
        if (spxeKeyDown(O)) {
            dK -= dT;
        }

        memcpy(pixbuf, sky, spxe.scrres.width * spxe.scrres.height * sizeof(Px));
        if (trail) {
            universe_dust_render(&universe, pixbuf, cam, scale);
        }

        if (!paused) {
            if (gravity) {
                universe_apply_gravity(&universe);
            }
            universe_update(&universe, clicked ? hover : -1, dT * dK);
        }

        hover = universe_render(&universe, pixbuf, cam, m, scale);
        mouse = m;
    }

    free(sky);
    universe_free(&universe);
    return spxeEnd(pixbuf);
}
