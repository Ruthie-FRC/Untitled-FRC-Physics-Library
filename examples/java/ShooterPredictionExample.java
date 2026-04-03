import frcsim_physics.PhysicsBody;
import frcsim_physics.PhysicsWorld;
import frcsim_physics.Vec3;

public final class ShooterPredictionExample {
	private ShooterPredictionExample() {}

	public static void main(String[] args) {
		try (PhysicsWorld world = new PhysicsWorld(0.01, true)) {
			PhysicsBody note = world.createBody(0.24);
			note.setPosition(new Vec3(0.0, 0.0, 1.0));
			note.setLinearVelocity(new Vec3(14.0, 0.0, 4.0));

			final int steps = 200;
			for (int i = 0; i < steps; ++i) {
				world.step();
				Vec3 pos = note.position();
				Vec3 vel = note.linearVelocity();
				if (i % 20 == 0) {
					System.out.printf("t=%.2fs pos=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f)%n",
							i * 0.01, pos.x(), pos.y(), pos.z(), vel.x(), vel.y(), vel.z());
				}

				if (pos.z() <= 0.0) {
					System.out.printf("Projectile hit ground at t=%.2fs, range=%.2fm%n", i * 0.01, pos.x());
					break;
				}
			}
		}
	}
}
