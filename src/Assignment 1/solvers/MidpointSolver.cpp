#include "MidpointSolver.h"

#include "Simulator.h"
#include "GlobalVars.h"
#include "util.h"

MidpointSolver::MidpointSolver(Solver *_solver) {
    if (_solver) {
        solver = _solver;
    }
}

void MidpointSolver::simulation_step(State *state, double dt) {
    if (!midpoint_state) {
        midpoint_state = new State(state, solver);
    } else {
        std::memcpy(midpoint_state->globals->data, state->globals->data, state->globals->size * sizeof(double));
    }

    //Util::PrintGlobals(midpoint_state->globals, "MSGlobals before.");
    midpoint_state->advance(0.5 * dt);
    //Util::PrintGlobals(midpoint_state->globals, "MSGlobals after.");

    for (int i = 0; i < 2 * state->globals->n; i++) {
        state->globals->x[i] += dt * midpoint_state->globals->v[i];
        state->globals->v[i] += dt * midpoint_state->globals->Q[i];
    }
}

SympleticMidpointSolver::SympleticMidpointSolver(Solver *_solver) {
    if (_solver) {
        solver = _solver;
    }
}

void SympleticMidpointSolver::simulation_step(State *state, double dt) {
    if (!midpoint_state) {
        midpoint_state = new State(state, solver);
    } else {
        std::memcpy(midpoint_state->globals->data, state->globals->data, state->globals->size * sizeof(double));
    }

    //Util::PrintGlobals(midpoint_state->globals, "MSGlobals before.");
    midpoint_state->advance(0.5 * dt);
    //Util::PrintGlobals(midpoint_state->globals, "MSGlobals after.");

    for (int i = 0; i < 2 * state->globals->n; i++) {
        state->globals->v[i] += dt * midpoint_state->globals->Q[i];
        state->globals->x[i] += dt * state->globals->v[i];
    }
}