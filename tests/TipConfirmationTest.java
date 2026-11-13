import org.firstinspires.ftc.teamcode.robot.util.TipConfirmation;

public final class TipConfirmationTest {
    private static void expect(boolean value, boolean expected) {
        if (value != expected) throw new AssertionError("Unexpected tip decision");
    }

    public static void main(String[] args) {
        TipConfirmation check = new TipConfirmation();
        expect(check.isConfirmed(0), false);
        check.observe(false, 0);
        expect(check.isConfirmed(0), false);
        for (int i = 1; i < 8; i++) check.observe(false, i * 100_000_000L);
        expect(check.isConfirmed(700_000_000L), true);
        expect(check.isConfirmed(1_100_000_000L), false); // stalled camera
        check.observe(true, 800_000_000L);
        for (int i = 9; i < 20; i++) check.observe(false, i * 100_000_000L);
        expect(check.isConfirmed(1_900_000_000L), false); // visible target veto
        check.reset();
        for (int i = 0; i < 20; i++) check.observe(false, 0);
        expect(check.isConfirmed(0), false); // duplicate timestamp
        check.reset();
        for (int i = 0; i < 8; i++) check.observe(false, i * 1_000_000L);
        expect(check.isConfirmed(7_000_000L), false); // insufficient duration
        check.observe(false, 1_000_000_000L);
        expect(check.isConfirmed(1_000_000_000L), false); // gap resets evidence
        check.reset();
        expect(check.isConfirmed(0), false);
        System.out.println("PASS: confirmation, dropout, veto, stale/duplicate frames, reset");
    }
}
