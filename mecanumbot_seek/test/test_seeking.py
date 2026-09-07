"""
Tests for the modelled SEEKING circuit.

The circuit is the research content of this package, so the tests are written
as claims about Panksepp's system rather than as coverage of the arithmetic:
that it is engaged by anticipation, that it outlasts the stimulus, that it
modulates what the robot perceives, that it falls under non-reward, and that it
switches off on consummation. If one of these fails, the model has stopped being
the thing it says it is.
"""

import pytest

from mecanumbot_seek.seeking import (
    APPROACH,
    CONSUMMATORY,
    DIRECTED,
    EXTINGUISHED,
    PHASES,
    UNDIRECTED,
    SeekingDrive,
)


def drive(**kwargs):
    """Build a circuit with the packaged shape, overridden as a test needs."""
    return SeekingDrive(**kwargs)


def run(circuit, seconds, dt=0.1):
    """Tick a circuit forward with no events, as the tree would between them."""
    steps = int(seconds / dt)
    for _ in range(steps):
        circuit.step(dt)
    return circuit


class TestEngagement:
    """How the circuit is switched on."""

    def test_an_undirected_circuit_wants_nothing_in_particular(self):
        # Panksepp: "this system does not 'want' anything specific before
        # learning; it just wants opportunities to explore the world." That is
        # T1, and it runs at the baseline rather than at nothing.
        circuit = drive()
        assert circuit.phase == UNDIRECTED
        assert not circuit.directed
        assert circuit.arousal == pytest.approx(circuit.baseline)

    def test_naming_an_object_directs_it(self):
        circuit = drive()
        circuit.reset("mug")
        assert circuit.phase == DIRECTED
        assert circuit.directed
        assert circuit.expectancy == pytest.approx(circuit.initial_expectancy)

    def test_a_cue_raises_arousal(self):
        circuit = drive()
        circuit.reset("mug")
        before = circuit.arousal
        circuit.cue(1.0)
        circuit.step(0.1)
        assert circuit.arousal > before

    def test_a_sighting_is_the_strongest_input(self):
        cued, seen = drive(), drive()
        for circuit in (cued, seen):
            circuit.reset("mug")
        cued.cue(1.0)
        seen.sight(1.0)
        cued.step(0.1)
        seen.step(0.1)
        assert seen.arousal > cued.arousal

    def test_a_sighting_also_raises_expectancy(self):
        # Seeing the thing is direct evidence it is findable, which is a claim
        # about the world and belongs to the slow layer, not only the fast one.
        circuit = drive()
        circuit.reset("mug")
        circuit.non_reward()
        circuit.non_reward()
        lowered = circuit.expectancy
        circuit.sight(1.0)
        assert circuit.expectancy > lowered

    def test_a_sighting_moves_it_to_the_approach_phase(self):
        circuit = drive()
        circuit.reset("mug")
        circuit.sight(1.0)
        assert circuit.phase == APPROACH
        assert circuit.object_in_sight

    def test_every_phase_is_one_of_the_declared_ones(self):
        circuit = drive()
        circuit.reset("mug")
        for action in (
            lambda: circuit.sight(1.0),
            lambda: circuit.non_reward(),
            lambda: circuit.consummate(),
        ):
            action()
            circuit.step(0.1)
            assert circuit.phase in PHASES


class TestSustainedArousal:
    """Panksepp's fourth attribute: emotions outlast the stimuli."""

    def test_arousal_survives_the_object_going_out_of_sight(self):
        # The property that makes the robot keep working near where it glimpsed
        # the object instead of losing interest the instant the detector drops.
        circuit = drive(arousal_tau=8.0)
        circuit.reset("mug")
        circuit.sight(1.0)
        circuit.step(0.1)
        circuit.out_of_sight()
        run(circuit, 4.0)
        assert circuit.arousal > 0.5

    def test_arousal_does_decay_eventually(self):
        circuit = drive(arousal_tau=8.0)
        circuit.reset("mug")
        circuit.sight(1.0)
        circuit.step(0.1)
        circuit.out_of_sight()
        run(circuit, 60.0)
        assert circuit.arousal == pytest.approx(circuit.baseline, abs=0.02)

    def test_losing_sight_is_not_itself_a_cost(self):
        # Losing sight of something is not evidence it has gone. Only a
        # completed fruitless search is.
        circuit = drive()
        circuit.reset("mug")
        before = circuit.expectancy
        circuit.out_of_sight()
        assert circuit.expectancy == pytest.approx(before)

    def test_the_baseline_is_a_floor_not_a_target(self):
        circuit = drive(baseline=0.1)
        circuit.reset("mug")
        run(circuit, 300.0)
        assert circuit.arousal >= 0.1

    def test_the_decay_is_independent_of_the_tick_period(self):
        # A behaviour tree's tick period is nominal, not guaranteed. A slow tick
        # under load must not silently change every rate in the model.
        fine, coarse = drive(), drive()
        for circuit in (fine, coarse):
            circuit.reset("mug")
            circuit.sight(1.0)
            circuit.step(0.01)
        run(fine, 5.0, dt=0.01)
        run(coarse, 5.0, dt=0.5)
        assert fine.arousal == pytest.approx(coarse.arousal, abs=0.02)


class TestIncentiveSalience:
    """Panksepp's third attribute: the circuit modulates sensory input."""

    def test_a_strongly_seeking_robot_accepts_a_weaker_detection(self):
        circuit = drive()
        circuit.reset("mug")
        calm = circuit.detection_threshold()
        circuit.sight(1.0)
        circuit.step(0.1)
        assert circuit.detection_threshold() < calm

    def test_the_threshold_is_bounded_at_both_ends(self):
        # A robot that will accept anything is not motivated, it is broken.
        circuit = drive(detection_threshold_high=0.7, detection_threshold_low=0.35)
        circuit.reset("mug")
        for _ in range(50):
            circuit.sight(1.0)
            circuit.step(0.1)
        assert circuit.detection_threshold() >= 0.35 - 1e-9
        run(circuit, 300.0)
        assert circuit.detection_threshold() <= 0.7 + 1e-9

    def test_the_threshold_moves_with_arousal_not_expectancy(self):
        # Salience is a moment-to-moment thing; belief that the object is
        # findable is not what makes a marginal frame convincing.
        circuit = drive()
        circuit.reset("mug")
        circuit.non_reward()
        after_disappointment = circuit.detection_threshold()
        circuit.expectancy = 0.9
        assert circuit.detection_threshold() == pytest.approx(after_disappointment)


class TestNonReward:
    """Expectancy falls on frustrative non-reward -- an event, not a prediction."""

    def test_a_fruitless_sweep_lowers_expectancy(self):
        circuit = drive(non_reward_cost=0.25)
        circuit.reset("mug")
        before = circuit.expectancy
        circuit.non_reward()
        assert circuit.expectancy == pytest.approx(before - 0.25)

    def test_non_rewards_are_counted(self):
        circuit = drive()
        circuit.reset("mug")
        for _ in range(3):
            circuit.non_reward()
        assert circuit.non_rewards == 3

    def test_enough_non_rewards_extinguish_the_episode(self):
        circuit = drive(
            initial_expectancy=0.8, non_reward_cost=0.25, extinction_threshold=0.15
        )
        circuit.reset("mug")
        assert not circuit.extinguished
        for _ in range(3):
            circuit.non_reward()
        assert circuit.extinguished
        assert circuit.phase == EXTINGUISHED

    def test_a_sighting_can_bring_it_back_from_extinction(self):
        # Extinction is a state, not a latch. Seeing the thing after giving up
        # on it re-engages the circuit, which is what an animal does.
        circuit = drive(initial_expectancy=0.8, non_reward_cost=0.3)
        circuit.reset("mug")
        for _ in range(3):
            circuit.non_reward()
        assert circuit.extinguished
        circuit.sight(1.0)
        circuit.step(0.1)
        assert not circuit.extinguished
        assert circuit.phase == APPROACH

    def test_expectancy_never_goes_negative(self):
        circuit = drive()
        circuit.reset("mug")
        for _ in range(20):
            circuit.non_reward()
        assert circuit.expectancy == pytest.approx(0.0)


class TestSearchWidth:
    """What falling expectancy does to the search."""

    def test_full_expectancy_searches_tight(self):
        circuit = drive(search_radius_min=1.5, search_radius_max=6.0)
        circuit.reset("mug")
        circuit.expectancy = 1.0
        assert circuit.search_radius() == pytest.approx(1.5)

    def test_no_expectancy_searches_wide(self):
        circuit = drive(search_radius_min=1.5, search_radius_max=6.0)
        circuit.reset("mug")
        circuit.expectancy = 0.0
        assert circuit.search_radius() == pytest.approx(6.0)

    def test_each_fruitless_sweep_widens_the_next_one(self):
        circuit = drive()
        circuit.reset("mug")
        widths = []
        for _ in range(3):
            widths.append(circuit.search_radius())
            circuit.non_reward()
        assert widths == sorted(widths)
        assert widths[-1] > widths[0]


class TestConsummation:
    """SEEKING is appetitive: it terminates on receipt."""

    def test_grasping_switches_the_circuit_off(self):
        circuit = drive()
        circuit.reset("mug")
        circuit.sight(1.0)
        circuit.step(0.1)
        circuit.consummate()
        assert circuit.arousal == pytest.approx(0.0)
        assert circuit.expectancy == pytest.approx(0.0)
        assert circuit.phase == CONSUMMATORY

    def test_it_stays_off_however_long_the_tree_keeps_ticking(self):
        # A robot still seeking with the thing in its grabbers is the model
        # being wrong, not the robot being keen. The baseline floor must not
        # bring it back.
        circuit = drive()
        circuit.reset("mug")
        circuit.consummate()
        run(circuit, 60.0)
        assert circuit.arousal == pytest.approx(0.0)
        assert circuit.phase == CONSUMMATORY

    def test_a_new_episode_starts_the_circuit_over(self):
        circuit = drive()
        circuit.reset("mug")
        circuit.consummate()
        circuit.reset("bottle")
        assert circuit.phase == DIRECTED
        assert circuit.object_class == "bottle"
        assert circuit.expectancy == pytest.approx(circuit.initial_expectancy)


class TestSnapshot:
    """What gets published."""

    def test_the_snapshot_carries_every_message_field(self):
        circuit = drive()
        circuit.reset("mug")
        snapshot = circuit.snapshot()
        assert set(snapshot) == {
            "object_class",
            "arousal",
            "expectancy",
            "object_in_sight",
            "seconds_since_cue",
            "search_radius",
            "phase",
        }

    def test_seconds_since_cue_counts_from_the_last_event(self):
        circuit = drive()
        circuit.reset("mug")
        run(circuit, 3.0)
        assert circuit.seconds_since_cue == pytest.approx(3.0, abs=0.15)
        circuit.sight(1.0)
        assert circuit.seconds_since_cue == pytest.approx(0.0)


class TestBounds:
    """Both layers stay in [0, 1] whatever they are fed."""

    @pytest.mark.parametrize("confidence", [0.0, 0.5, 1.0, 5.0, -1.0])
    def test_arousal_stays_bounded(self, confidence):
        circuit = drive()
        circuit.reset("mug")
        for _ in range(50):
            circuit.sight(confidence)
            circuit.step(0.1)
        assert 0.0 <= circuit.arousal <= 1.0
        assert 0.0 <= circuit.expectancy <= 1.0

    def test_a_zero_dt_changes_nothing(self):
        circuit = drive()
        circuit.reset("mug")
        before = (circuit.arousal, circuit.expectancy)
        circuit.step(0.0)
        assert (circuit.arousal, circuit.expectancy) == pytest.approx(before)

    def test_a_negative_dt_is_treated_as_zero(self):
        # Clock jumps happen; time must not run backwards through the model.
        circuit = drive()
        circuit.reset("mug")
        before = (circuit.arousal, circuit.expectancy)
        circuit.step(-5.0)
        assert (circuit.arousal, circuit.expectancy) == pytest.approx(before)
