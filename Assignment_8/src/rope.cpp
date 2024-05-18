#include <cassert>
#include <cstddef>
#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
        assert(num_nodes>0);
        masses.reserve(num_nodes);
        springs.reserve(num_nodes);
        masses.emplace_back(new Mass(start,node_mass,false));
        for(int i=1;i<num_nodes;++i){
            masses.emplace_back(new Mass(start + i * 1.0f/num_nodes * (end-start),node_mass,false));
            springs.emplace_back(new Spring(masses[i-1],masses[i],k));
        }
        for(auto& i:pinned_nodes){
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto sp_ab = s->m1->position - s->m2->position;
            auto sp_ba = s->m2->position - s->m1->position;
            auto sp_ab_inv = 1/sp_ab.norm();
            auto sp_ba_inv = 1/sp_ba.norm();
            s->m1->forces += -s->k * sp_ab * sp_ab_inv * (sp_ab.norm() - s->rest_length);
            s->m2->forces += -s->k * sp_ba * sp_ba_inv * (sp_ba.norm() - s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                auto a = m->forces/m->mass + gravity;
                m->velocity += a*delta_t;
                m->position = m->velocity*delta_t;// For semi-implicit method
                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto sp_ab = s->m1->position - s->m2->position;
            auto sp_ba = s->m2->position - s->m1->position;
            auto sp_ab_inv = 1/sp_ab.norm();
            auto sp_ba_inv = 1/sp_ba.norm();
            s->m1->forces += -s->k * sp_ab * sp_ab_inv * (sp_ab.norm() - s->rest_length);
            s->m2->forces += -s->k * sp_ba * sp_ba_inv * (sp_ba.norm() - s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                auto a = m->forces/m->mass + gravity;
                auto last_pos = m->position;
                m->position = m->position + (1-damping_factor) * (m->position - m->last_position) + a*delta_t*delta_t;
                m->last_position = last_pos;
                // TODO (Part 4): Add global Verlet damping
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
