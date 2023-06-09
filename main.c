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
#define G 0.0667F
#define THETA 0.5F
#define FRICTION 0.1F
#define MASS_MIN 1.0F
#define MASS_MAX 0.0F
#define TRAIL_POOL_SIZE 32
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
    vec2 center;
    float mass;
};

struct QuadTree {
    struct QuadNode* root;
    struct QuadArray outsiders;
};

struct Trail {
    vec2 positions[TRAIL_POOL_SIZE];
    float timers[TRAIL_POOL_SIZE];
    int marker;
};

struct Body {
    vec2 p;
    vec2 v;
    float r;
    float sqr;
    Px color;
    struct QuadNode* nodes[4];
    struct Trail trail;
};

struct Universe {
    size_t count;
    struct Body* bodies;
    struct QuadTree quadtree;
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

static vec2 vec2_mouse_position(void)
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

static void circle_render(
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

static void circle_render_outlined(
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

static int rect_point_overlap(const Rect rect, const vec2 p)
{
    return (p.x <= (rect.p.x + rect.q.x) && p.x >= (rect.p.x - rect.q.x) &&
            p.y <= (rect.p.y + rect.q.y) && p.y >= (rect.p.y - rect.q.y));
}

static int rect_overlap(const Rect r1, const Rect r2)
{
    return ((r1.p.x + r1.q.x) <= (r2.p.x - r2.q.x) && 
            (r1.p.x - r1.q.x) >= (r2.p.x + r2.q.x) &&
            (r1.p.y + r1.q.y) <= (r2.p.y - r2.q.y) && 
            (r1.p.y - r1.q.y) >= (r2.p.y + r2.q.y));
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

static size_t quadarray_collision_check(const struct QuadArray* qarray, 
    const struct Universe* universe, const size_t index, const vec2 p, 
    const float r, const float dT)
{
    for (size_t i = 0; i < qarray->size; ++i) {
        const size_t n = qarray->indices[i];
        if (n != index) {
            const vec2 q = vec2_delta(
                universe->bodies[n].p, universe->bodies[n].v, dT
            );
            if (circle_overlap(p, q, r, universe->bodies[n].r)) {
                return n;
            }
        } 
    }
    return -1;
}

static void quadnode_push(struct QuadNode*, struct Universe*, size_t);
static void body_node_push(struct Body*, struct QuadNode*);
static void body_node_remove(struct Body*, const struct QuadNode*);

static struct QuadNode quadnode_create(const vec2 p, const vec2 q)
{
    struct QuadNode qnode;
    qnode.rect = (Rect){p, q};
    qnode.elements = quadarray_create();
    qnode.children = NULL;
    qnode.active = 1;
    qnode.center = p;
    qnode.mass = 0.0F;
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
        body_node_remove(universe->bodies +  k, qnode);
        for (int i = 0; i < 4; ++i) {
            qnode->children[i].active = 1;
            if (rect_circle_overlap(qnode->children[i].rect, 
                universe->bodies[k].p, universe->bodies[k].r)) {
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
            body_node_remove(universe->bodies + k, child);
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
    const float r = universe->bodies[index].r;
    const vec2 p = universe->bodies[index].p;
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
        body_node_push(universe->bodies + index, qnode);
    }
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

static void quadnode_update(struct QuadNode* qnode, const struct Universe* universe)
{
    float m = 0.0F;
    vec2 p = {0.0F, 0.0F};
    
    if (qnode->active) {
        const size_t count = qnode->elements.size;
        for (size_t i = 0; i < count; ++i) {
            m += universe->bodies[qnode->elements.indices[i]].sqr;
        }

        const float div = 1.0F / m;
        for (size_t i = 0; i < count; ++i) {
            const size_t k = qnode->elements.indices[i];
            const float d = universe->bodies[k].sqr * div;
            p.x += universe->bodies[k].p.x * d;
            p.y += universe->bodies[k].p.y * d;
        }
    } else {
        for (size_t i = 0; i < 4; ++i) {
            quadnode_update(qnode->children + i, universe);
            m += qnode->children[i].mass;
        }

        const float div = 1.0F / m;
        for (size_t i = 0; i < 4; ++i) {
            const float d = qnode->children[i].mass * div;
            p.x += qnode->children[i].center.x * d;
            p.y += qnode->children[i].center.y * d;
        }
    }

    qnode->mass = m;
    qnode->center = p;
}

static void quadnode_render(const struct QuadNode* qnode, Px* pixbuf, 
    const vec2 cam, const vec2 hres, float scale)
{
    if (!qnode->active) {
        for (int i = 0; i < 4; ++i) {
            quadnode_render(qnode->children + i, pixbuf, cam, hres, scale);
        }

        /*const Px green = {0, 255, 0, 255};
        const float R = sqrtf(qnode->mass) * scale;
        const vec2 P = vec2_world_to_screen(qnode->center, cam, hres, scale);
        circle_render_outlined(pixbuf, P, R, R * R, green, (R - 2.0F) * (R - 2.0F));*/
    } else {
        Px c = qnode->elements.size ? (Px){255, 0, 0, 255} : (Px){0, 0, 255, 255};
        const vec2 p = vec2_world_to_screen(qnode->rect.p, cam, hres, scale);
        const vec2 q = {qnode->rect.q.x * scale, qnode->rect.q.y * scale};
        rect_render(pixbuf, (Rect){p, q}, c);

        /*const float R = sqrtf(qnode->mass) * scale;
        const vec2 P = vec2_world_to_screen(qnode->center, cam, hres, scale);
        circle_render_outlined(pixbuf, P, R, R * R, c, (R - 2.0F) * (R - 2.0F));*/
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
        quadtree->root->rect, universe->bodies[index].p, universe->bodies[index].r)) {
        quadarray_push_if(&quadtree->outsiders, index);
    }

    if (rect_circle_overlap(
        quadtree->root->rect, universe->bodies[index].p, universe->bodies[index].r)) {
        quadnode_push(quadtree->root, universe, index);
    }
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

static struct QuadTree quadtree_create_universe(struct Universe* universe, const float n)
{
    const vec2 hres = {(float)spxe.scrres.width * 0.5, (float)spxe.scrres.height * 0.5};
    const vec2 nres = {hres.x * n, hres.y * n};
    struct QuadTree quadtree = quadtree_create(hres, nres);
    const size_t count = universe->count;
    for (size_t i = 0; i < count; ++i) {
        quadtree_push(&quadtree, universe, i);
    }
    return quadtree;
}

static void body_node_push(struct Body* body, struct QuadNode* node)
{
    for (size_t i = 0; i < 4; ++i) {
        if (!body->nodes[i]) {
            body->nodes[i] = node;
            break;
        } else if (body->nodes[i] == node) {
            break;
        }
    }
}

static void body_node_remove(struct Body* body, const struct QuadNode* node)
{
    if (body->nodes[3] == node) {
        body->nodes[3] = NULL;
    } else {
        for (size_t i = 0; i < 3; ++i) {
            if (body->nodes[i] == node) {
                memmove(
                    body->nodes + i,
                    body->nodes + i + 1,
                    (3 - i) * sizeof(struct QuadNode*)
                );
                body->nodes[3] = NULL;
                break;
            }
        }
    }
}

static vec2 body_quadarray_gravity(const struct QuadArray* qarray, const Rect rect,
    const struct Universe* universe, const vec2 p, const float g)
{
    vec2 F = {0.0F, 0.0F};
    for (size_t i = 0; i < qarray->size; ++i) {
        const size_t k = qarray->indices[i];
        const float dx = p.x - universe->bodies[k].p.x;
        const float dy = p.y - universe->bodies[k].p.y;
        if (dx != 0.0F && dy != 0.0F && 
            rect_point_overlap(rect, universe->bodies[k].p)) {
            const float f = g * universe->bodies[k].sqr / (dx * dx + dy * dy);
            F.x += f * dx;
            F.y += f * dy;
        }
    }
    return F;
}

static vec2 body_quadnode_gravity(const struct QuadNode* qnode,
    const struct Universe* universe, const vec2 p, const float g)
{
    vec2 F;
    const float dx = p.x - qnode->center.x;
    const float dy = p.y - qnode->center.y;
    const float sqdist = (dx * dx + dy * dy);
    const float sqcenter =  qnode->rect.q.x * qnode->rect.q.y;
    if (sqdist && sqcenter / sqdist < THETA) {
        const float f = g * qnode->mass / sqdist;
        F = (vec2) {f * dx, f * dy};
    } else if (qnode->active) {
        F = body_quadarray_gravity(&qnode->elements, qnode->rect, universe, p, g);
    } else {
        F = (vec2){0.0F, 0.0F};
        for (size_t i = 0; i < 4; ++i) {
            vec2 f = body_quadnode_gravity(qnode->children + i, universe, p, g);
            F.x += f.x;
            F.y += f.y;
        }
    }
    
    return F;
}

static vec2 body_gravity(
    const struct Universe* universe, const vec2 p, const float g)
{
    static const Rect r = {{0.0F, 0.0F}, {1.175494351e38, 1.175494351e38}};
    vec2 outF = body_quadarray_gravity(&universe->quadtree.outsiders, r, universe, p, g);
    vec2 inF = body_quadnode_gravity(universe->quadtree.root, universe, p, g);
    return (vec2){outF.x + inF.x, outF.y + inF.y};
}



static int body_spawn(
    const struct Universe* universe, const vec2 p, const float r, const size_t index)
{
    for (size_t i = 0; i < index; ++i) {
        if (circle_overlap(p, universe->bodies[i].p, r, universe->bodies[i].r)) {
            return 1;
        }
    }
    return 0;
}

static float universe_spawn(struct Universe* universe, const float speed)
{
    float ret = 0.0F;
    const int width = spxe.scrres.width, height = spxe.scrres.height;
    const vec2 hscr = {(float)width * 0.5F, (float)height * 0.5F};
    const float scale = (float)(width < height ? width : height) / 30.0F;
    const float minr = scale * MASS_MIN, maxr = scale * MASS_MAX;
    const size_t count = universe->count;
    for (size_t i = 0; i < count; ++i) {
        float n = 1.0F;
        universe->bodies[i].p.x = frand() * (float)width;
        universe->bodies[i].p.y = frand() * (float)height;
        universe->bodies[i].r = minr + frand() * maxr;
        while (body_spawn(
            universe, universe->bodies[i].p, universe->bodies[i].r, i)) {
            float m = n * 2.0F;
            universe->bodies[i].p.x = (m * frand() - n + 0.5F) * (float)width;
            universe->bodies[i].p.y = (m * frand() - n + 0.5F) * (float)height;
            universe->bodies[i].r = minr + frand() * maxr;
            n = m;
        }
        
        ret = n > ret ? n : ret;
        universe->bodies[i].v = (vec2){
            -(hscr.y - universe->bodies[i].p.y) * speed,
            (hscr.x - universe->bodies[i].p.x) * speed
        };

        universe->bodies[i].sqr = universe->bodies[i].r * universe->bodies[i].r;
        universe->bodies[i].color.r = rand() % 256;
        universe->bodies[i].color.g = rand() % 256;
        universe->bodies[i].color.b = rand() % 256;
        universe->bodies[i].color.a = 255;
        memset(&universe->bodies[i].trail, 0, sizeof(struct Trail));
    }

    return ret;
}

static struct Universe universe_create(const size_t count, const float speed)
{
    struct Universe universe;
    universe.count = count;
    universe.bodies = calloc(count, sizeof(*universe.bodies));
    const float n = universe_spawn(&universe, speed);
    universe.quadtree = quadtree_create_universe(&universe, n);
    return universe;
}

static void universe_free(struct Universe* universe)
{
    if (universe->count) {
        free(universe->bodies);
        quadtree_free(&universe->quadtree);
        memset(universe, 0, sizeof(struct Universe));
    }
}

static void universe_apply_gravity(const struct Universe* universe, const float g)
{
    for (size_t i = 0; i < universe->count; ++i) {
        const vec2 F = body_gravity(universe, universe->bodies[i].p, g);
        universe->bodies[i].v.x -= F.x;
        universe->bodies[i].v.y -= F.y;
    }
}

static void universe_collision_check(
    const struct Universe* universe, const size_t i, const float dT)
{
    const float r = universe->bodies[i].r;
    const vec2 p = vec2_delta(universe->bodies[i].p, universe->bodies[i].v, dT);
    const size_t j = quadtree_collision_check(universe, i, p, r, dT);

    if (j != (size_t)-1) {
        const vec2 q = vec2_delta(universe->bodies[j].p, universe->bodies[j].v, dT);
        const float mass = universe->bodies[i].sqr + universe->bodies[j].sqr;
        const float dx = q.x - p.x;
        const float dy = q.y - p.y;
        const float d = sqrtf(dx * dx + dy * dy);
        const vec2 n = {dx / d, dy / d};
        const float P = (universe->bodies[i].v.x * n.x + 
            universe->bodies[i].v.y * n.y - universe->bodies[j].v.x * n.x
            - universe->bodies[j].v.y * n.y) * (2.0F - FRICTION) / mass;

        universe->bodies[i].v.x -= P * universe->bodies[j].sqr * n.x;
        universe->bodies[i].v.y -= P * universe->bodies[j].sqr * n.y;
        universe->bodies[j].v.x += P * universe->bodies[i].sqr * n.x;
        universe->bodies[j].v.y += P * universe->bodies[i].sqr * n.y;

        vec2 c1 = vec2_delta(universe->bodies[i].p, universe->bodies[i].v, dT); 
        vec2 c2 = vec2_delta(universe->bodies[j].p, universe->bodies[j].v, dT); 
        
        if (circle_overlap(c1, c2, universe->bodies[i].r, universe->bodies[j].r)) {
            float R = r + universe->bodies[j].r;
            float k1 = r / R;
            float k2 = 1.0F - k1;
            vec2 d = {c2.x - c1.x, c2.y - c1.y};
            vec2 n = vec2_norm(d);
            vec2 v = {d.x - n.x * R, d.y - n.y * R};
            universe->bodies[i].p.x += k1 * v.x;
            universe->bodies[i].p.y += k1 * v.y;
            universe->bodies[j].p.x -= k2 * v.x;
            universe->bodies[j].p.y -= k2 * v.y;
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
            universe->bodies[i].p.x += universe->bodies[i].v.x * dT;
            universe->bodies[i].p.y += universe->bodies[i].v.y * dT;
        }
        
        for (size_t j = 0; j < 4 && universe->bodies[i].nodes[j]; ++j) {
            if (!rect_circle_overlap(universe->bodies[i].nodes[j]->rect,
                universe->bodies[i].p, universe->bodies[i].r)) {
                quadarray_remove_if(&universe->bodies[i].nodes[j]->elements, i);
                body_node_remove(universe->bodies + i, universe->bodies[i].nodes[j]);
            }
        }
        quadtree_push(&universe->quadtree, universe, i);
    }
    
    if (universe->quadtree.root->children) {
        quadnode_reduce(universe->quadtree.root, universe);
    }
    quadnode_update(universe->quadtree.root, universe);
}

static void universe_trails_render(
    const struct Universe* universe, Px* pixbuf, const vec2 cam, const float scale)
{
    const int w = spxe.scrres.width, h = spxe.scrres.height;
    const vec2 hres = {(float)w * 0.5F, (float)h * 0.5F};
    for (size_t i = 0; i < universe->count; ++i) { 
        const int marker = universe->bodies[i].trail.marker;
        for (size_t j = 0; j < TRAIL_POOL_SIZE; ++j) {
            if (universe->bodies[i].trail.timers[j] > 0.0F) {
                const vec2 dq = vec2_world_to_screen(
                    universe->bodies[i].trail.positions[j], cam, hres, scale
                );
                
                const int x = (int)dq.x;
                const int y = (int)dq.y;
                if (x >= 0 && y >= 0 && x < w && y < h) {
                    pixbuf[y * w + x] = pxlerp(
                        pixbuf[y * w + x],
                        universe->bodies[i].color,
                        universe->bodies[i].trail.timers[j]
                    );
                }
                universe->bodies[i].trail.timers[j] -= 1.0F / (float)TRAIL_POOL_SIZE;
            }
        }
        universe->bodies[i].trail.positions[marker] = universe->bodies[i].p;
        universe->bodies[i].trail.timers[marker] = 1.0F;
        universe->bodies[i].trail.marker = (marker + 1) % TRAIL_POOL_SIZE;
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
        float r = universe->bodies[i].r * scale;
        float sqr = r * r;
        const vec2 q = vec2_world_to_screen(universe->bodies[i].p, cam, hres, scale);
        if (q.x + r > 0.0F && q.y + r > 0.0F && q.x - r < (float)w && q.y - r < (float)h){
            if (mouse_overlap == -1 && circle_point_overlap(q, mouse, sqr)) {
                float t = r - 2.0F;
                Px n = {
                    255 - universe->bodies[i].color.r, 
                    255 - universe->bodies[i].color.g,
                    255 - universe->bodies[i].color.b, 
                    255
                };
                circle_render_outlined(pixbuf, q, r, sqr, n, t * t);
                mouse_overlap = i;
            } else {
                circle_render(pixbuf, q, r, sqr,  universe->bodies[i].color);
            }
        }
    }
    return mouse_overlap;
}

static Px* gsim_background_create(const int total, const int prob)
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
    printf("-s <float>\t: Set initial angular speed of the system. Default is %f\n",.0F);
    printf("-help\t\t: Print information about usage of gsim.\n");
    return EXIT_SUCCESS;
}

static void gsim_print_controls(void)
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
    printf("Q:\tSwitch the rendering of the quadtree on and off.\n");
    printf("R:\tRestart the simulation.\n");
    printf("Escape:\tQuit.\n");
}

int main(const int argc, const char** argv)
{
    Px* pixbuf, *sky;
    struct Universe universe;
    int i, count, width = WIDTH, height = HEIGHT;
    int gravity = 1, paused = 0, hover = -1, trail = 0, qtree = 0;
    float t, T, dT, dK = 1.0F, scale = 1.0F, g = G, speed = 0.0F;
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
    sky = gsim_background_create(1000, 2);
    gsim_print_controls();

    const vec2 hres = {(float)spxe.scrres.width * 0.5, (float)spxe.scrres.height * 0.5};
    t = spxeTime();
    
    while (spxeRun(pixbuf)) {
        T = spxeTime();
        dT = T - t;
        t = T;
        
        const float dM = dT * 100.0F;
        const vec2 m = vec2_mouse_position();
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
                vec2 p = universe.bodies[hover].p;
                vec2 q = {p.x - d.x / scale, p.y - d.y / scale};
                universe.bodies[hover].v.x = (q.x - p.x) / (dT * dK);
                universe.bodies[hover].v.y = (q.y - p.y) / (dT * dK);
                universe.bodies[hover].p = q;
                if (spxeKeyDown(V)) {
                    float r = universe.bodies[hover].r - (dT * dK) * 10.0F;
                    universe.bodies[hover].r = r > 0.0001 ? r : universe.bodies[hover].r;
                    universe.bodies[hover].sqr = r * r;
                }
                if (spxeKeyDown(B)) {
                    float r = universe.bodies[hover].r + (dT * dK) * 10.0F;
                    universe.bodies[hover].r = r > 0.0001 ? r : universe.bodies[hover].r;
                    universe.bodies[hover].sqr = r * r;
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

