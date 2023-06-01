/*

Copyright (c) 2023 Eugenio Arteaga A.

Permission is hereby granted, free of charge, to any 
person obtaining a copy of this software and associated 
documentation files (the "Software"), to deal in the 
Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to 
permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice 
shall be included in all copies or substantial portions
of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

/************************ gsim **************************/

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
#define DUST_POOL_SIZE 100
#define G 0.0667F
#define S 0.0F
#define MASS_MIN 1.0F
#define MASS_MAX 1.0F
#define QUADNODE_MAXSIZE 16
#define QUADNODE_MINSIZE (QUADNODE_MAXSIZE / 4)
#define QUADNODE_MAXLEN 100.0F

typedef struct vec2 {
    float x, y;
} vec2;

typedef struct Rect {
    vec2 p, q;
} Rect;

struct QuadArray {
    size_t size;
    size_t capacity;
    size_t* indices;
};

struct QuadNode {
    int active;
    Rect rect;
    struct QuadArray elements;
    struct QuadNode* children;
};

struct QuadTree {
    struct QuadNode* root;
    struct QuadArray outsiders;
};

struct Trail {
    vec2 positions[DUST_POOL_SIZE];
    float timers[DUST_POOL_SIZE];
    int marker;
};

struct Universe {
    size_t count;
    vec2* positions;
    vec2* velocities;
    struct Trail* trails;
    float* radiuses;
    float* sqradiuses;
    Px* colors;
    struct QuadTree quadtree;
    struct QuadNode** nodes;
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

static vec2 vec2_world_to_screen(vec2 p, vec2 cam, vec2 hres, float scale)
{
    return (vec2){
        hres.x + (hres.x - p.x + cam.x) * scale, 
        hres.y + (hres.y - p.y + cam.y) * scale
    };
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

static int rect_circle_overlap(const Rect rect, const vec2 p, const float r)
{ 
    return ((p.x - r) <= (rect.p.x + rect.q.x) && (p.x + r) >= (rect.p.x - rect.q.x) &&
            (p.y - r) <= (rect.p.y + rect.q.y) && (p.y + r) >= (rect.p.y - rect.q.y));
}

static int rect_circle_outside(const Rect rect, const vec2 p, const float r)
{
    return (p.x + r > rect.p.x + rect.q.x || p.x - r < rect.p.x - rect.q.x ||
            p.y + r > rect.p.y + rect.q.y || p.y - r < rect.p.y - rect.q.y);
}

static int rect_overlap(const Rect r1, const Rect r2)
{
    return ((r1.p.x + r1.q.x) <= (r2.p.x - r2.q.x) && 
            (r1.p.x - r1.q.x) >= (r2.p.x + r2.q.x) &&
            (r1.p.y + r1.q.y) <= (r2.p.y - r2.q.y) && 
            (r1.p.y - r1.q.y) >= (r2.p.y + r2.q.y));
}

static int universe_spawn_collision(const struct Universe* universe, const size_t index)
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

static float universe_spawn(struct Universe* universe, const float speed)
{
    float ret = 0.0F;
    int width = spxe.scrres.width, height = spxe.scrres.height;
    const vec2 hscr = {(float)width * 0.5F, (float)height * 0.5F};
    const float scale = (float)(width < height ? width : height) / 30.0F;
    const float minr = scale * MASS_MIN, maxr = scale * MASS_MAX;
    const size_t count = universe->count;
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
        
        ret = n > ret ? n : ret;
        universe->velocities[i] = (vec2){
            -(hscr.y - universe->positions[i].y) * speed,
            (hscr.x - universe->positions[i].x) * speed
        };

        universe->sqradiuses[i] = universe->radiuses[i] * universe->radiuses[i];
        universe->colors[i].r = rand() % 256;
        universe->colors[i].g = rand() % 256;
        universe->colors[i].b = rand() % 256;
        universe->colors[i].a = 255;
        memset(universe->trails + i, 0, sizeof(struct Trail));
    }

    return ret;
}

static struct QuadArray quadarray_create(void)
{
    struct QuadArray qarray;
    qarray.size = 0;
    qarray.capacity = 0;
    qarray.indices = NULL;
    return qarray;
}

static void quadarray_free(struct QuadArray* qarray)
{
    if (qarray->indices) {
        free(qarray->indices);
        qarray->indices = NULL;
        qarray->size = 0;
        qarray->capacity = 0;
    }
}

static void quadarray_push(struct QuadArray* qarray, const size_t element)
{
    if (qarray->size + 1 >= qarray->capacity) {
        qarray->capacity = (!qarray->capacity + qarray->capacity) * 2;
        qarray->indices = realloc(
            qarray->indices, qarray->capacity * sizeof(*qarray->indices)
        );
    }
    qarray->indices[qarray->size++] = element;
}

static void quadarray_push_if(struct QuadArray* qarray, const size_t element)
{
    const size_t count = qarray->size;
    for (size_t i = 0; i < count; ++i) {
        if (qarray->indices[i] == element) {
            return;
        }
    }
    quadarray_push(qarray, element);
}

static void quadarray_remove(struct QuadArray* qarray, const size_t index)
{
    if (index == qarray->size - 1) {
        --qarray->size;
    } else {
        memmove(
            qarray->indices + index, 
            qarray->indices + index + 1, 
            (--qarray->size - index) * sizeof(*qarray->indices)
        );
    }
}

static void quadarray_remove_if(struct QuadArray* qarray, const size_t element)
{
    const size_t count = qarray->size;
    for (size_t i = 0; i < count; ++i) {
        if (qarray->indices[i] == element) {
            quadarray_remove(qarray, i);
            break;
        }
    }
}

static struct QuadNode quadnode_create(const vec2 p, const vec2 q)
{
    struct QuadNode qnode;
    qnode.rect = (Rect){p, q};
    qnode.elements = quadarray_create();
    qnode.children = NULL;
    qnode.active = 1;
    return qnode;
}

static void quadnode_free(struct QuadNode* qnode)
{
    quadarray_free(&qnode->elements);
    if (qnode->children) {
        for (int i = 0; i < 4; ++i) {
            quadnode_free(qnode->children + i);
        }
        free(qnode->children);
        qnode->children = NULL;
    }
}

static void nodepush(struct QuadNode** nodes, struct QuadNode* node)
{
    for (size_t i = 0; i < 4; ++i) {
        if (!nodes[i]) {
            nodes[i] = node;
            break;
        } else if (nodes[i] == node) {
            break;
        }
    }
}

static void noderemove(struct QuadNode** nodes, const struct QuadNode* node)
{
    if (nodes[3] == node) {
        nodes[3] = NULL;
    } else {
        for (size_t i = 0; i < 3; ++i) {
            if (nodes[i] == node) {
                memmove(nodes + i, nodes + i + 1, (3 - i) * sizeof(struct QuadNode*));
                nodes[3] = NULL;
                break;
            }
        }
    }
}

static void quadnode_push(struct QuadNode*, struct Universe*, size_t);

static void quadnode_split(struct QuadNode* qnode, struct Universe* universe)
{
    if (!qnode->children) {
        const vec2 p = qnode->rect.p;
        const vec2 q = {qnode->rect.q.x * 0.5F, qnode->rect.q.y * 0.5F};
        qnode->children = malloc(sizeof(*qnode->children) * 4);
        qnode->children[0] = quadnode_create((vec2){p.x - q.x, p.y + q.y}, q);
        qnode->children[1] = quadnode_create((vec2){p.x + q.x, p.y + q.y}, q);
        qnode->children[2] = quadnode_create((vec2){p.x + q.x, p.y - q.y}, q);
        qnode->children[3] = quadnode_create((vec2){p.x - q.x, p.y - q.y}, q);
    }
    
    while (qnode->elements.size) {
        const size_t k = qnode->elements.indices[--qnode->elements.size];
        noderemove(universe->nodes + k * 4, qnode);
        for (int i = 0; i < 4; ++i) {
            qnode->children[i].active = 1;
            if (rect_circle_overlap(qnode->children[i].rect, 
                universe->positions[k], universe->radiuses[k])) {
                quadnode_push(qnode->children + i, universe, k);
            }
        }
    }

    qnode->active = 0;
}

static void quadnode_merge(struct QuadNode* qnode, struct Universe* universe)
{
    qnode->active = 1;
    for (int i = 0; i < 4; ++i) {
        struct QuadNode* child = qnode->children + i;
        child->active = 0;
        while (child->elements.size) {
            const size_t k = child->elements.indices[--child->elements.size];
            const size_t n = k * 4;
            noderemove(universe->nodes + n, child);
            quadnode_push(qnode, universe, k);
        }
    }
}

static void quadnode_reduce(struct QuadNode* qnode, struct Universe* universe)
{
    if (qnode->children[0].active && 
        qnode->children[0].elements.size < QUADNODE_MINSIZE &&
        qnode->children[1].active &&
        qnode->children[1].elements.size < QUADNODE_MINSIZE &&
        qnode->children[2].active &&
        qnode->children[2].elements.size < QUADNODE_MINSIZE &&
        qnode->children[3].active &&
        qnode->children[3].elements.size < QUADNODE_MINSIZE) {
        quadnode_merge(qnode, universe);
    } else {
        for (int i = 0; i < 4; ++i) {
            if (qnode->children[i].children) {
                quadnode_reduce(qnode->children + i, universe);
            }
        }
    }
}

static void quadnode_push(
    struct QuadNode* qnode, struct Universe* universe, size_t index)
{
    const float r = universe->radiuses[index];
    const vec2 p = universe->positions[index];
    if (!qnode->active && qnode->children) {
        for (int i = 0; i < 4; ++i) {
            if (rect_circle_overlap(qnode->children[i].rect, p, r)) {
                quadnode_push(qnode->children + i, universe, index);
            }
        }
    } else if (qnode->elements.size + 1 > QUADNODE_MAXSIZE &&
        qnode->rect.q.x > QUADNODE_MAXLEN && qnode->rect.q.y > QUADNODE_MAXLEN) {
        quadnode_split(qnode, universe);
        quadnode_push(qnode, universe, index);
    } else {
        quadarray_push_if(&qnode->elements, index);
        nodepush(universe->nodes + index * 4, qnode);
    }
}

static struct QuadTree quadtree_create(const vec2 p, const vec2 q)
{
    struct QuadTree quadtree;
    quadtree.root = malloc(sizeof(*quadtree.root));
    *quadtree.root = quadnode_create(p, q);
    quadtree.outsiders = quadarray_create();
    return quadtree;
}

static void quadtree_free(struct QuadTree* quadtree)
{
    quadnode_free(quadtree->root);
    quadarray_free(&quadtree->outsiders);
    free(quadtree->root);
    memset(quadtree, 0, sizeof(*quadtree));
}

static void quadtree_push(
    struct QuadTree* quadtree, struct Universe* universe, size_t index)
{
    if (rect_circle_outside(
        quadtree->root->rect, universe->positions[index], universe->radiuses[index])) {
        quadarray_push_if(&quadtree->outsiders, index);
    }

    if (rect_circle_overlap(
        quadtree->root->rect, universe->positions[index], universe->radiuses[index])) {
        quadnode_push(quadtree->root, universe, index);
    }
}

static size_t quadarray_collision_check(const struct QuadArray* qarray, 
    const struct Universe* universe, const size_t index, const vec2 p, 
    const float r, const float dT)
{
    for (size_t i = 0; i < qarray->size; ++i) {
        const size_t n = qarray->indices[i];
        if (n != index) {
            const vec2 q = vec2_delta(
                universe->positions[n], universe->velocities[n], dT
            );
            if (circle_overlap(p, q, r, universe->radiuses[n])) {
                return n;
            }
        } 
    }
    return -1;
}

static size_t quadnode_collision_check(const struct QuadNode* qnode,
    const struct Universe* universe, const size_t index, const vec2 p,
    const float r, const float dT)
{
    if (qnode->active) {
        return quadarray_collision_check(&qnode->elements, universe, index, p, r, dT);
    }

    for (int i = 0; i < 4; ++i) {
        if (rect_circle_overlap(qnode->children[i].rect, p, r)) {
            size_t collision = quadnode_collision_check(
                qnode->children + i, universe, index, p, r, dT
            );
            if (collision != (size_t)-1) {
                return collision;
            }
        }
    }

    return -1;
}

static size_t quadtree_collision_check(const struct Universe* universe, 
    const size_t index, const vec2 p, const float r, const float dT)
{
    size_t collision;
    const struct QuadTree* quadtree = &universe->quadtree;
    if (rect_circle_outside(quadtree->root->rect, p, r)) {
        collision = quadarray_collision_check(
            &quadtree->outsiders, universe, index, p, r, dT
        );
        if (collision != (size_t)-1) {
            return collision;
        }
    }
   
    if (rect_circle_overlap(quadtree->root->rect, p, r)) {
        return quadnode_collision_check(
            quadtree->root, universe, index, p, r, dT
        );
    }

    return -1;
}

static struct QuadTree universe_quadtree_create(struct Universe* universe, const float n)
{
    const vec2 hres = {(float)spxe.scrres.width * 0.5F, (float)spxe.scrres.height * 0.5F};
    const vec2 nres = {hres.x * n, hres.y * n};
    struct QuadTree quadtree = quadtree_create(hres, nres);
    const size_t count = universe->count;
    for (size_t i = 0; i < count; ++i) {
        quadtree_push(&quadtree, universe, i);
    }
    return quadtree;
}

static struct Universe universe_create(const size_t count, const float speed)
{
    struct Universe universe;
    universe.count = count;
    universe.positions = malloc(count * sizeof(vec2));
    universe.velocities = calloc(count, sizeof(vec2));
    universe.radiuses = malloc(count * sizeof(float));
    universe.sqradiuses = malloc(count * sizeof(float));
    universe.colors = malloc(count * sizeof(Px));
    universe.trails = calloc(count, sizeof(struct Trail));
    universe.nodes = calloc(count * 4, sizeof(struct QuadNode*));
    const float n = universe_spawn(&universe, speed);
    universe.quadtree = universe_quadtree_create(&universe, n);
    return universe;
}

static void universe_free(struct Universe* universe)
{
    if (universe->count) {
        free(universe->positions);
        free(universe->velocities);
        free(universe->radiuses);
        free(universe->sqradiuses);
        free(universe->colors);
        free(universe->trails);
        free(universe->nodes);
        quadtree_free(&universe->quadtree);
        memset(universe, 0, sizeof(struct Universe));
    }
}

static void universe_apply_gravity(const struct Universe* universe, const float g)
{
    for (size_t i = 0; i < universe->count; ++i) {
        for (size_t j = i + 1; j < universe->count; ++j) {
            const float dx = universe->positions[i].x - universe->positions[j].x;
            const float dy = universe->positions[i].y - universe->positions[j].y;
            if (dx != 0.0F && dy != 0.0F) {
                const float f = g / (dx * dx + dy * dy);
                const float fi = f * universe->sqradiuses[j];
                const float fj = f * universe->sqradiuses[i];
                universe->velocities[i].x -= fi * dx;
                universe->velocities[i].y -= fi * dy;
                universe->velocities[j].x += fj * dx;
                universe->velocities[j].y += fj * dy;
            }
        }
    }
}

static void universe_collision_check(
    const struct Universe* universe, const size_t i, const float dT)
{
    const float r = universe->radiuses[i];
    const vec2 p = vec2_delta(universe->positions[i], universe->velocities[i], dT);
    const size_t j = quadtree_collision_check(universe, i, p, r, dT);

    if (j != (size_t)-1) {
        const vec2 q = vec2_delta(universe->positions[j], universe->velocities[j], dT);
        const float mass = universe->sqradiuses[i] + universe->sqradiuses[j];
        const float dx = q.x - p.x;
        const float dy = q.y - p.y;
        const float d = sqrtf(dx * dx + dy * dy);
        const vec2 n = {dx / d, dy / d};
        const float P = (universe->velocities[i].x * n.x + 
            universe->velocities[i].y * n.y - universe->velocities[j].x * n.x
            - universe->velocities[j].y * n.y) * 1.98F / mass;

        universe->velocities[i].x -= P * universe->sqradiuses[j] * n.x;
        universe->velocities[i].y -= P * universe->sqradiuses[j] * n.y;
        universe->velocities[j].x += P * universe->sqradiuses[i] * n.x;
        universe->velocities[j].y += P * universe->sqradiuses[i] * n.y;

        vec2 c1 = vec2_delta(universe->positions[i], universe->velocities[i], dT); 
        vec2 c2 = vec2_delta(universe->positions[j], universe->velocities[j], dT); 
        
        if (circle_overlap(c1, c2, universe->radiuses[i], universe->radiuses[j])) {
            float R = r + universe->radiuses[j];
            float k1 = r / R;
            float k2 = 1.0F - k1;
            vec2 d = {c2.x - c1.x, c2.y - c1.y};
            vec2 n = vec2_norm(d);
            vec2 v = {d.x - n.x * R, d.y - n.y * R};
            universe->positions[i].x += k1 * v.x;
            universe->positions[i].y += k1 * v.y;
            universe->positions[j].x -= k2 * v.x;
            universe->positions[j].y -= k2 * v.y;
        } else {
            universe_collision_check(universe, j, dT);
            universe_collision_check(universe, i, dT);
        }
    }
}

static void universe_update(
    struct Universe* universe, const int hover, const float dT)
{
    const size_t h = (size_t)hover;
    for (size_t i = 0; i < universe->count; ++i) {
        if (h != i) {
            universe_collision_check(universe, i, dT);
        }
    }
    
    for (size_t i = 0; i < universe->count; ++i) { 
        if (h != i) {
            universe->positions[i].x += universe->velocities[i].x * dT;
            universe->positions[i].y += universe->velocities[i].y * dT;
        }
        
        const size_t n = i * 4;
        for (size_t j = 0; j < 4 && universe->nodes[n + j]; ++j) {
            if (!rect_circle_overlap(universe->nodes[n + j]->rect, 
                universe->positions[i], universe->radiuses[i])) {
                quadarray_remove_if(&universe->nodes[n + j]->elements, i);
                noderemove(universe->nodes + n, universe->nodes[n + j]);
            }
        }
        quadtree_push(&universe->quadtree, universe, i);
    }

    if (universe->quadtree.root->children) {
        quadnode_reduce(universe->quadtree.root, universe);
    }
}

static void rect_render(Px* pixbuf, const Rect rect, const Px c)
{
    const int width = spxe.scrres.width, height = spxe.scrres.height;
    const int startx = clampf(rect.p.x - rect.q.x, 0.0F, (float)(width - 1));
    const int starty = clampf(rect.p.y - rect.q.y, 0.0F, (float)(height - 1));
    const int endx = clampf(rect.p.x + rect.q.x + 1.0F, 0.0F, (float)(width - 1));
    const int endy = clampf(rect.p.y + rect.q.y + 1.0F, 0.0F, (float)(height - 1));
    const int startw = starty * width, endw = endy * width;
    
    for (int x = startx; x <= endx; ++x) {
        pixbuf[startw + x] = c;
        pixbuf[endw + x] = c;
    }

    for (int y = (starty + 1) * width; y < endw; y += width) {
        pixbuf[y + startx] = c;
        pixbuf[y + endx] = c;
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

static void universe_trails_render(
    const struct Universe* universe, Px* pixbuf, const vec2 cam, const float scale)
{
    const int w = spxe.scrres.width, h = spxe.scrres.height;
    const vec2 hres = {(float)w * 0.5F, (float)h * 0.5F};
    for (size_t i = 0; i < universe->count; ++i) { 
        for (size_t j = 0; j < DUST_POOL_SIZE; ++j) {
            if (universe->trails[i].timers[j] > 0.0F) {
                const vec2 dq = vec2_world_to_screen(
                    universe->trails[i].positions[j], cam, hres, scale
                );
                
                const int x = (int)dq.x;
                const int y = (int)dq.y;
                if (x >= 0 && y >= 0 && x < w && y < h) {
                    pixbuf[y * w + x] = pxlerp(
                        pixbuf[y * w + x],
                        universe->colors[i],
                        universe->trails[i].timers[j]
                    );
                }
                universe->trails[i].timers[j] -= 1.0F / (float)DUST_POOL_SIZE;
            }
        }
        universe->trails[i].positions[universe->trails[i].marker] = universe->positions[i];
        universe->trails[i].timers[universe->trails[i].marker] = 1.0F;
        universe->trails[i].marker = (universe->trails[i].marker + 1) % DUST_POOL_SIZE;
    }
}

static void quadnode_render(const struct QuadNode* qnode, Px* pixbuf, 
    const vec2 cam, const vec2 hres, float scale)
{
    if (!qnode->active) {
        for (int i = 0; i < 4; ++i) {
            quadnode_render(qnode->children + i, pixbuf, cam, hres, scale);
        }
    } else {
        Px c = qnode->elements.size ? (Px){255, 0, 0, 255} : (Px){0, 0, 255, 255};
        const vec2 p = vec2_world_to_screen(qnode->rect.p, cam, hres, scale);
        const vec2 q = {qnode->rect.q.x * scale, qnode->rect.q.y * scale};
        rect_render(pixbuf, (Rect){p, q}, c);
    }
}

static int universe_render(
    const struct Universe* universe,    Px* pixbuf,             const vec2 cam, 
    const vec2 mouse,                   const float scale)
{
    int mouse_overlap = -1;
    const int w = spxe.scrres.width, h = spxe.scrres.height;
    const vec2 hres = {(float)w * 0.5F, (float)h * 0.5F};
    for (size_t i = 0; i < universe->count; ++i) { 
        float r = universe->radiuses[i] * scale;
        float sqr = r * r;
        const vec2 q = vec2_world_to_screen(universe->positions[i], cam, hres, scale);
        if (q.x + r > 0.0F && q.y + r > 0.0F && q.x - r < (float)w && q.y - r < (float)h){
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

static int gsim_error(const char* fmt, const char* arg)
{
    fprintf(stderr, "gsim: ");
    fprintf(stderr, fmt, arg);
    fprintf(stderr, " See -help for more information.\n");
    return EXIT_FAILURE;
}

static int gsim_help(void)
{
    printf("gsim usage:\n");
    printf("<uint>\t\t: Set number of bodies to simulate in the system.\n");
    printf("-w <uint>\t: Set width of the rendered simulation in pixels.\n");
    printf("-h <uint>\t: Set height of the rendered simulation in pixels.\n");
    printf("-g <float>\t: Set gravitational constant value. Default is %f\n", G);
    printf("-s <float>\t: Set initial angular speed of the system. Default is %f\n", S);
    printf("-help\t\t: Print information about usage of gsim.\n");
    return EXIT_SUCCESS;
}

static void gsim_controls(void)
{
    printf("gsim controls:\n");
    printf("Click and drag on the background to move around.\n");
    printf("Click and drag on the bodies of the simulation to move them.\n");
    printf("WASD:\tMove up, down, left and right.\n");
    printf("Z - X:\tZoom in and out of the center of the screen.\n");
    printf("O - P:\tIncrease and decrease the size of the time step.\n");
    printf("G:\tTurn gravity on and off.\n");
    printf("Space:\tPause the time of the physics in the simulation.\n");
    printf("LShift:\tSwitch the rendering of trails on and off.\n");
    printf("R:\tRestart the simulation.\n");
    printf("Escape:\tQuit.\n");
}

int main(const int argc, const char** argv)
{
    Px* pixbuf, *sky;
    struct Universe universe;
    int i, count, width = WIDTH, height = HEIGHT;
    int gravity = 1, paused = 0, hover = -1, trail = 1, qtree = 1;
    float t, T, dT, dK = 1.0F, scale = 1.0F, g = G, speed = S;
    vec2 cam = {0.0F, 0.0F}, mouse = {0.0F, 0.0F};
    
    srand(time(NULL));
    count = rand() % 10 + 5;
    
    for (i = 1; i < argc; ++i) {
        int *inum = NULL;
        float *fnum = NULL;
        if (!strcmp(argv[i], "-help")) {
            return gsim_help();
        } else if (!strcmp(argv[i], "-w")) {
            inum = &width;
        } else if (!strcmp(argv[i], "-h")) {
            inum = &height;
        } else if (!strcmp(argv[i], "-g")) {
            fnum = &g;
        } else if (!strcmp(argv[i], "-s")) {
            fnum = &speed;
        } else if (argv[i][0] >= '0' && argv[i][0] <= '9') {
            count = atoi(argv[i]);
        } else {
            return gsim_error("illegal option '%s'.", argv[i]);
        }

        if (!inum && !fnum) {
            continue;
        }

        if (i + 1 == argc) {
            return gsim_error("expected numeric argument after option '%s'.", argv[i]);
        }
        
        if (inum) {
            *inum = atoi(argv[i + 1]);
            if (*inum < 1) {
                 return gsim_error(
                    "argument for option '%s' must be a non-zero unsigned integer.", 
                    argv[i]
                );
            }
        } else {
            *fnum = atof(argv[i + 1]);
        }
        ++i;
    }

    pixbuf = spxeStart("gsim", WIDTH, HEIGHT, width, height);
    universe = universe_create(count, speed);
    sky = universe_background_create(1000, 2);
    gsim_controls();

    const vec2 hres = {(float)spxe.scrres.width * 0.5, (float)spxe.scrres.height * 0.5};
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
            universe_spawn(&universe, speed);
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
        if (spxeKeyPressed(Q)) {
            qtree = !qtree;
        }

        if (clicked) {
            vec2 d = {m.x - mouse.x, m.y - mouse.y};
            if (hover != -1) {
                vec2 p = universe.positions[hover];
                vec2 q = {p.x - d.x / scale, p.y - d.y / scale};
                universe.velocities[hover].x = (q.x - p.x) / (dT * dK);
                universe.velocities[hover].y = (q.y - p.y) / (dT * dK);
                universe.positions[hover] = q;
                if (spxeKeyDown(V)) {
                    float r = universe.radiuses[hover] - (dT * dK) * 10.0F;
                    universe.radiuses[hover] = r > 0.0001 ? r : universe.radiuses[hover];
                    universe.sqradiuses[hover] = r * r;
                }
                if (spxeKeyDown(B)) {
                    float r = universe.radiuses[hover] + (dT * dK) * 10.0F;
                    universe.radiuses[hover] = r > 0.0001 ? r : universe.radiuses[hover];
                    universe.sqradiuses[hover] = r * r;
                }
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
            universe_trails_render(&universe, pixbuf, cam, scale);
        }

        if (!paused) {
            if (gravity) {
                universe_apply_gravity(&universe, g);
            }
            universe_update(&universe, clicked ? hover : -1, dT * dK);
        }

        if (qtree) {
            quadnode_render(universe.quadtree.root, pixbuf, cam, hres, scale);
        }

        hover = universe_render(&universe, pixbuf, cam, m, scale);
        mouse = m;
    }

    free(sky);
    universe_free(&universe);
    return spxeEnd(pixbuf);
}

