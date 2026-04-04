package rensim.simulation.projectile;

import rensim.Vec3;

/**
 * Launch plan output from ballistic solver.
 */
public record ProjectileLaunchPlan(Vec3 velocityMps, double timeOfFlightSeconds) {}
