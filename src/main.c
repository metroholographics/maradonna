#include "raylib.h"
#include "raymath.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "main.h"

Leg_Element thigh;
Leg_Element leg;
Leg_Element foot;
Joint_Element hip;
Joint_Element knee;
Joint_Element ankle;
Joint_Element toe;

Ball ball;

Joint_Element* joints_array[JOINT_COUNT];
Leg_Element* legs_array[3];

int selected_joint;

Vector2 get_leg_origin(Leg_Element* l)
{
    Vector2 v = (Vector2) {.x = l->shape.width, .y = 0};
    return v;
}

Leg_Element make_leg_element(Joint_Element* origin, float width, float height)
{
    Leg_Element l;
    l.origin = origin;
    l.shape.width = width;
    l.shape.height = height;
    l.rotation = 0.0f;

    l.shape.x = l.origin->centre_position.x;
    l.shape.y = l.origin->centre_position.y;

    l.centre_pos = (Vector2) {
        .x = l.shape.x + (0.5f * l.shape.width),
        .y = l.shape.y + (0.5f * l.shape.height)
    };
    l.color = RED;
    l.selected = false;
    l.leg_points = (Leg_Points) {0};

    return l;
}

Joint_Element make_joint_element(Leg_Element* from, Leg_Element* to, float radius)
{
    Joint_Element j;
    j = (Joint_Element) {
        .connects_from = from,
        .connects_to = to,
        .radius = radius,
    };
    if (from != NULL) {
        Vector2 origin_offset = get_leg_origin(from);
         j.centre_position = (Vector2) {
            .x = from->shape.x + origin_offset.x,
            .y = from->shape.y + origin_offset.y
        };
    }

    return j;
}

void select_joint(Joint_Element** joints)
{
    if (!IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) return;

    Vector2 mouse_pos = GetMousePosition();

    for (int i = 1; i < 4; i++) {
        if (joints[i]->connects_from == NULL) continue;
        if (CheckCollisionPointCircle(mouse_pos, joints[i]->centre_position, JOINT_RADIUS)) {
            if (selected_joint != -1) {
                joints_array[selected_joint]->connects_from->selected = false;
            }
            joints[i]->connects_from->selected = true;
            selected_joint = i;
            return;
        }
    }
    if (selected_joint != -1) {
        joints_array[selected_joint]->connects_from->selected = false;
        selected_joint = -1;
    }
}

void move_leg(Leg_Element* l) 
{
    Vector2 mouse_d = GetMouseDelta();
    //printf("x %f y %f\n", mouse_d.x, mouse_d.y);
    if (mouse_d.y > 0) {
        l->rotation -= 0.25f * mouse_d.y;
    } else if (mouse_d.y < 0) {
        l->rotation += 0.25 * -mouse_d.y;
    }
}

void handle_leg_elements(Leg_Element** legs)
{
    for (int i = 0; i < 3; i++) {
        if (legs[i]->selected) {
            legs[i]->color = BLUE;
            move_leg(legs[i]);
        } else {
            legs[i]->color = RED;
        }
    }
}

void solve_leg_chain(Vector2 target, Joint_Element** joints, int joint_count)
{
    Vector2 positions[JOINT_COUNT];
    for (int i = 0; i < joint_count; i++) {
        positions[i] = joints[i]->centre_position;
    }
    Vector2 fixed_start = positions[0];
    Vector2 t = target;

    float lengths[JOINT_COUNT - 1];
    float total_length = 0.0f;
    for (int i = 0; i < joint_count - 1; i++) {
        Leg_Element* l = joints[i]->connects_to;
        float len = Vector2Length((Vector2) {l->shape.width, l->shape.height});
        lengths[i] = len;
        total_length += len;
    }

    float len_to_target = Vector2Length(Vector2Subtract(t, positions[0]));
    if (len_to_target > total_length) {
        Vector2 dir = Vector2Subtract(t, positions[2]);
        dir = Vector2Normalize(dir);
        positions[JOINT_COUNT - 1] = Vector2Add(Vector2Scale(dir, lengths[2]), positions[2]);
    }
    
    int i_k_count = 0;
    for (int i = 0; i < IK_ITERATIONS; i++) {
        positions[JOINT_COUNT - 1] = t;
        //Backwards
        for (int i = JOINT_COUNT - 2; i >= 0; i--) {
            Vector2 dir = Vector2Subtract(positions[i], positions[i + 1]);
            dir = Vector2Normalize(dir);
            positions[i] = Vector2Add(positions[i+1], Vector2Scale(dir, lengths[i]));
        }
        //forwards
        positions[0] = fixed_start;
        for (int i = 1; i < joint_count - 1; i++) {
            Vector2 dir = Vector2Subtract(positions[i], positions[i - 1]);
            dir = Vector2Normalize(dir);
            positions[i] = Vector2Add(positions[i - 1], Vector2Scale(dir, lengths[i - 1]));
        }
        i_k_count++;
    }

    for (int i = 0; i < JOINT_COUNT; i++) {
        joints[i]->centre_position = positions[i];
    }

}

Vector2 get_rotated_end(Leg_Element l)
{
    float dx = -l.shape.width;
    float dy = l.shape.height;
    float angle = DEG2RAD * l.rotation;
    float rx = dx * cosf(angle) - dy * sinf(angle);
    float ry = dx * sinf(angle) + dy * cosf(angle);
    return (Vector2) {rx, ry};
}

void update_joint_positions(Joint_Element** joints)
{
    for (int i = 0; i < JOINT_COUNT; i++) 
    {
        if (i == 0) continue;
        Leg_Element* parent = joints[i]->connects_from;
        Leg_Element* l = parent;
        float angle = DEG2RAD * l->rotation;
        l->leg_points.top_right = (Vector2) {l->shape.x, l->shape.y};
        Vector2 tr = l->leg_points.top_right;
        l->leg_points.top_left = (Vector2) {
            .x = tr.x + -l->shape.width * cosf(angle) - 0 * sinf(angle),
            .y = tr.y + -l->shape.width * sinf(angle) + 0 * cosf(angle)
        };
        l->leg_points.bot_left = (Vector2) {
            .x = tr.x + -l->shape.width * cosf(angle) - l->shape.height * sinf(angle),
            .y = tr.y + -l->shape.width * sinf(angle) + l->shape.height * cosf(angle)
        };
        l->leg_points.bot_right = (Vector2) {
            .x = tr.x + 0 * cosf(angle) - l->shape.height * sinf(angle),
            .y = tr.y + 0 * sinf(angle) + l->shape.height * cosf(angle)
        };

        Vector2 rotated = get_rotated_end(*parent);

        joints[i]->centre_position.x = parent->shape.x + rotated.x;
        joints[i]->centre_position.y = parent->shape.y + rotated.y;
        Leg_Element* ll = joints[i]->connects_to;
        if (ll != NULL) {
            ll->shape.x = joints[i]->centre_position.x;
            ll->shape.y = joints[i]->centre_position.y;
        }
    }
}

void rotate_legs(Joint_Element** joints)
{
    for (int i = 0; i < JOINT_COUNT - 1; i++) {
        Leg_Element* l = joints[i]->connects_to;
        if (l == NULL) continue;
        float a = DEG2RAD * l->rotation;
        Vector2 centre_fixed = joints[i]->centre_position;
        Vector2 pointA2 = (Vector2) {
            .x = centre_fixed.x + (cosf(a) * l->shape.width),
            .y = centre_fixed.y + l->shape.height + (sinf(a) * l->shape.width),
        };
        Vector2 pointB = joints[i + 1]->centre_position;
        Vector2 diff = Vector2Subtract(pointB, pointA2);
        float angle = 180.0f + (RAD2DEG * atan2(diff.y, diff.x));
        l->rotation = angle;
        l->shape.x = joints[i]->centre_position.x;
        l->shape.y = joints[i]->centre_position.y;
    }
}

void update_ball(Ball* b, Leg_Element** legs, float dt)
{
    if (IsKeyPressed(KEY_SPACE)) {
        b->centre_position = (Vector2) {WIDTH/2, HEIGHT/2};
        b->velocity = (Vector2){0,0};
        b->acceleration = (Vector2){0,0};
        b->hit = false;
    }
    Vector2 ball_pos = b->centre_position;
    Vector2 vel = b->velocity;
    Vector2 acc = b->acceleration;

    if (!ball.hit) {
        acc.y = GRAVITY * dt;
    }

    //to-do: figure out velocity and acceleration relationship. I want the ball to stop if acc is 0

    for (int i = 0; i < LEG_COUNT; i++) {
        Leg_Points lp = legs[i]->leg_points;
        bool top_hit = CheckCollisionCircleLine(ball_pos, BALL_RADIUS, lp.top_left, lp.top_right);
        bool left_hit = CheckCollisionCircleLine(ball_pos, BALL_RADIUS, lp.top_left, lp.bot_left);
        bool bot_hit = CheckCollisionCircleLine(ball_pos, BALL_RADIUS, lp.bot_left, lp.bot_right);
        bool right_hit = CheckCollisionCircleLine(ball_pos, BALL_RADIUS, lp.top_right, lp.bot_right);

        if (top_hit || left_hit || bot_hit || right_hit) {
            printf("Hit! "); 
            ball.hit = true;
            vel.y = 0;
            acc.y = 0;
            break;
        }
    }
    printf("Vel: %f %f, Acc: %f %f\n",vel.x, vel.y, acc.x, acc.y);
    vel = Vector2Add(vel, acc);

    ball_pos.x += b->velocity.x;
    ball_pos.y += b->velocity.y;

    b->centre_position.x = ball_pos.x;
    b->centre_position.y = ball_pos.y;
    b->velocity = vel;
    b->acceleration = acc;
}


int main (int argc, char* argv[])
{
    (void)argc; (void)argv;

    InitWindow(WIDTH, HEIGHT, "maradonna");

    hip = make_joint_element(NULL, &thigh, JOINT_RADIUS);
    hip.centre_position = (Vector2) {WIDTH, 500};
    thigh = make_leg_element(&hip, 120.0f, 50.0f);
    knee = make_joint_element(&thigh, &leg, JOINT_RADIUS);
    leg = make_leg_element(&knee, 50.0f, 160.0f);
    ankle = make_joint_element(&leg, &foot, JOINT_RADIUS);
    foot = make_leg_element(&ankle, 75.0f, 30.0f);
    toe = make_joint_element(&foot, NULL, JOINT_RADIUS);
    ball = (Ball) {
        .centre_position = (Vector2){WIDTH * 0.5f, HEIGHT * 0.5f},
        .velocity = (Vector2){0,0},
        .acceleration = (Vector2){0,0},
        .radius = BALL_RADIUS,
        .hit = false
    };

    SetTargetFPS(60);
    selected_joint = -1;

    joints_array[0] = &hip;
    joints_array[1] = &knee;
    joints_array[2] = &ankle;
    joints_array[3] = &toe;
   
    legs_array[0] = &thigh;
    legs_array[1] = &leg;
    legs_array[2] = &foot;
    update_joint_positions(joints_array);
    
    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        select_joint(joints_array);

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && foot.selected) {
            Vector2 mouse = GetMousePosition();
            solve_leg_chain(mouse, joints_array, JOINT_COUNT);
            rotate_legs(joints_array);
        }

        update_joint_positions(joints_array);
        handle_leg_elements(legs_array);

        update_ball(&ball, legs_array, dt);

        BeginDrawing();
            ClearBackground(P_DARK_BLUE);
            DrawRectanglePro(thigh.shape, get_leg_origin(&thigh), thigh.rotation, thigh.color);
            DrawRectanglePro(leg.shape, get_leg_origin(&leg), leg.rotation, leg.color);
            DrawRectanglePro(foot.shape, get_leg_origin(&foot), foot.rotation, foot.color);
            DrawCircleV(hip.centre_position, hip.radius, GREEN);
            DrawCircleV(knee.centre_position, knee.radius, GREEN);
            DrawCircleV(ankle.centre_position, ankle.radius, GREEN);
            DrawCircleV(toe.centre_position, toe.radius, GREEN);
            for (int i = 0; i < 3; i++) {
                draw_leg_points(legs_array[i]);
                // DrawCircleV((Vector2){legs_array[i]->shape.x, legs_array[i]->shape.y}, 4, YELLOW); // leg origin
                // DrawCircleV(legs_array[i]->origin->centre_position, 4, ORANGE); // joint it connects to
            }
            DrawCircleV(ball.centre_position, ball.radius, LIGHTGRAY);
            // DrawLine(hip.centre_position.x, hip.centre_position.y, knee.centre_position.x, knee.centre_position.y, BLACK);
            // DrawLine(knee.centre_position.x, knee.centre_position.y, ankle.centre_position.x, ankle.centre_position.y, BLACK);
            // DrawLine(ankle.centre_position.x, ankle.centre_position.y, toe.centre_position.x, toe.centre_position.y, BLACK);
        EndDrawing();
    }

    CloseWindow();
}

void draw_leg_points(Leg_Element* l)
{
    DrawCircleV(l->leg_points.top_left, 4, BLACK);
    DrawCircleV(l->leg_points.top_right, 4, BLACK);
    DrawCircleV(l->leg_points.bot_left, 4, BLACK);
    DrawCircleV(l->leg_points.bot_right, 4, BLACK);
}