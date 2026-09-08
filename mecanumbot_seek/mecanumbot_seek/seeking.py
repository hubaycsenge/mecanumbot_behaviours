"""
The SEEKING circuit, as a state the tree carries and the behaviours read.

Panksepp's SEEKING is the appetitive foraging system: the one that makes an
animal leave where it is and go and look. Four of its properties are what this
module implements, and each of them changes what the robot actually does rather
than only what it reports.

**It is expectancy, not receipt.** SEEKING is engaged by the anticipation of
finding something and is switched off by finding it -- it is not a pleasure
system, and `consummate()` collapses it rather than raising it.

**It outlasts the stimulus.** Panksepp lists "positive feedback that sustains
arousal after the precipitating event has passed" as a defining attribute of an
emotional system, and it is why the robot keeps looking for a while after the
object goes out of sight rather than stopping the instant it does. Here that is
a long leak on the short-term layer.

**It modulates sensory input** -- incentive salience. A strongly seeking robot
accepts a weaker detection of the thing it is looking for, which is
`detection_threshold()` and is a real change to the perception the tree acts on,
not a metaphor.

**It wants nothing in particular before it is directed.** *"This system does not
'want' anything specific before learning; it just wants opportunities to explore
the world."* That is the `undirected` phase: with no object named the circuit
still runs, at the baseline. T2 is the same circuit with an incentive attached.

**T1 does not read it yet.** `mecanumbot_custom_nav2` explores on its own terms
and consumes nothing from here; whether an undirected circuit should steer the
frontier search is an open thesis question, recorded in this package's README.
Until it is answered, `undirected` is a state this model can be in rather than a
state that changes what the robot does.

> **This is deliberately not a reward-prediction error.** Panksepp rejects the
> RPE reduction explicitly and at length -- §7 of the 2005 paper is titled *"The
> SEEKING/expectancy/wanting system of the brain: It's not just 'reward
> prediction error'"* -- and the thesis wiki records that the SEEKING-dopamine-TD
> bridge "must not appear in the thesis in its original form". Nothing here
> compares a predicted value against a received one. `expectancy` falls under
> **frustrative non-reward**, an event, and the events are named as such.

## The formalism

Two layers, after the discrete state-space engine of Szabó et al. (2011), which
took its parallel-circuit structure from Panksepp in the first place:

    arousal[k+1]    = a * arousal[k]    + b * u[k]
    expectancy[k+1] = m * expectancy[k] + n * arousal[k]

`arousal` is the short-term layer -- how hard the robot is seeking right now.
`expectancy` is the medium-term layer, Szabó's "mood": how much the robot still
believes this object is findable at all. Both saturate to [0, 1].

The one departure is that `a` and `m` are given as **time constants in seconds**
and converted per tick with `exp(-dt / tau)`, rather than as fixed per-step
coefficients. A behaviour tree's tick period is nominal, not guaranteed -- a
slow tick under load would otherwise silently change every rate in the model.

Nothing here imports ROS, py_trees or numpy, so the whole circuit runs at a
desk and a trial's drive curve can be replayed off a bag.
"""

import math

# The phases a seeking episode passes through, published on `SeekingState`.
UNDIRECTED = "undirected"
DIRECTED = "directed"
APPROACH = "approach"
CONSUMMATORY = "consummatory"
EXTINGUISHED = "extinguished"

PHASES = (UNDIRECTED, DIRECTED, APPROACH, CONSUMMATORY, EXTINGUISHED)


def _clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


class SeekingDrive:
    """
    One SEEKING circuit: two coupled layers, driven by events, read by behaviours.

    Built once per tree and ticked from `TickSeekingDrive`. Every constant is a
    constructor argument taken from the constants YAML, so the shape of a run's
    drive curve is configuration and not code.
    """

    def __init__(
        self,
        # --- the two layers ---------------------------------------------------
        arousal_tau=8.0,
        expectancy_tau=90.0,
        expectancy_gain=0.02,
        baseline=0.1,
        # --- what events are worth --------------------------------------------
        sight_drive=1.0,
        cue_drive=0.5,
        arrival_drive=0.6,
        non_reward_cost=0.25,
        # --- what the drive is for --------------------------------------------
        search_radius_min=1.5,
        search_radius_max=6.0,
        detection_threshold_high=0.7,
        detection_threshold_low=0.35,
        extinction_threshold=0.15,
        # --- where it starts --------------------------------------------------
        initial_expectancy=0.8,
    ):
        self.arousal_tau = float(arousal_tau)
        self.expectancy_tau = float(expectancy_tau)
        self.expectancy_gain = float(expectancy_gain)
        self.baseline = float(baseline)

        self.sight_drive = float(sight_drive)
        self.cue_drive = float(cue_drive)
        self.arrival_drive = float(arrival_drive)
        self.non_reward_cost = float(non_reward_cost)

        self.search_radius_min = float(search_radius_min)
        self.search_radius_max = float(search_radius_max)
        self.detection_threshold_high = float(detection_threshold_high)
        self.detection_threshold_low = float(detection_threshold_low)
        self.extinction_threshold = float(extinction_threshold)
        self.initial_expectancy = float(initial_expectancy)

        self.reset()

    # --- lifecycle ------------------------------------------------------------

    def reset(self, object_class=""):
        """
        Start a fresh episode, directed at `object_class` or at nothing.

        With no object named the circuit is in `UNDIRECTED` -- engaged by the
        opportunity to explore rather than by any particular thing, which is
        what T1 runs on.
        """
        self.object_class = object_class or ""
        self.arousal = self.baseline
        self.expectancy = self.initial_expectancy if object_class else self.baseline
        self.object_in_sight = False
        self.seconds_since_cue = 0.0
        self.non_rewards = 0
        self._input = 0.0
        self._consummated = False
        self.phase = DIRECTED if object_class else UNDIRECTED

    # --- events ---------------------------------------------------------------
    #
    # Each of these is something that happened, not a number computed from a
    # prediction. That is the whole difference between this model and an RPE.

    def sight(self, confidence=1.0):
        """
        Register that the object is visible -- the unconditional input.

        The strongest event the circuit has.

        Panksepp's first defining attribute of an emotional system is
        unconditional sensory access -- the sight of the thing being sought
        engages the circuit without having to be learned. It also *raises*
        expectancy, because seeing it is direct evidence it is findable.
        """
        self._input += self.sight_drive * _clamp(confidence)
        self.expectancy = _clamp(self.expectancy + 0.5 * _clamp(confidence))
        self.object_in_sight = True
        self.seconds_since_cue = 0.0
        self.phase = APPROACH

    def out_of_sight(self):
        """
        Register that the object is no longer visible.

        Deliberately not an event with a cost: losing sight of something is not
        evidence it has gone, and the sustained-arousal property means the robot
        should keep looking where it was for a while. The decay does that on its
        own.
        """
        self.object_in_sight = False

    def cue(self, strength=1.0):
        """Register a weaker incentive than a sighting -- being told where it was."""
        self._input += self.cue_drive * _clamp(strength)
        self.seconds_since_cue = 0.0

    def arrive_at_expected(self):
        """
        Register reaching the place the object was last known to be.

        An incentive in its own right -- the anticipation peaks on arrival --
        and the moment before the question is answered either way.
        """
        self._input += self.arrival_drive
        self.seconds_since_cue = 0.0

    def non_reward(self):
        """
        Frustrative non-reward: the robot looked, and the object was not there.

        This is what lowers expectancy, and it is an *event* -- a completed
        sweep that found nothing, or arriving where the object was and finding
        an empty floor. Repeated enough, expectancy falls under
        `extinction_threshold` and the episode ends.
        """
        self.non_rewards += 1
        self.expectancy = _clamp(self.expectancy - self.non_reward_cost)
        if self.extinguished:
            self.phase = EXTINGUISHED

    def consummate(self):
        """
        Register that the object was grasped; SEEKING switches off.

        The appetitive system terminates on consummation; it is not a pleasure
        system, and a robot still seeking after it has the thing in its grabbers
        is the model being wrong rather than the robot being keen.
        """
        self.arousal = 0.0
        self.expectancy = 0.0
        self.object_in_sight = False
        self._consummated = True
        self.phase = CONSUMMATORY

    # --- the tick -------------------------------------------------------------

    def step(self, dt):
        """
        Advance both layers by `dt` seconds and clear the accumulated input.

        The leaks are `exp(-dt / tau)`, so an irregular tick period changes when
        the model is sampled but not how fast anything in it decays.
        """
        dt = max(0.0, float(dt))
        if self._consummated:
            self.seconds_since_cue += dt
            return self

        arousal_leak = math.exp(-dt / self.arousal_tau) if self.arousal_tau > 0 else 0.0
        expectancy_leak = (
            math.exp(-dt / self.expectancy_tau) if self.expectancy_tau > 0 else 0.0
        )

        # Short-term layer. The baseline is a floor rather than a term: SEEKING
        # is never entirely off in a waking animal, and a floor keeps that true
        # without the leak having to fight a constant input.
        self.arousal = _clamp(
            max(self.baseline, self.arousal * arousal_leak + self._input)
        )

        # Medium-term layer, driven by the short-term one. Slow by design: this
        # is the layer that has to survive a minute of fruitless searching and
        # still remember the robot was told the object is here.
        self.expectancy = _clamp(
            self.expectancy * expectancy_leak
            + self.expectancy_gain * self.arousal * dt
        )

        self._input = 0.0
        self.seconds_since_cue += dt
        if self.extinguished:
            self.phase = EXTINGUISHED
        elif self.phase == EXTINGUISHED:
            # An event put expectancy back above the floor -- a sighting, say.
            self.phase = APPROACH if self.object_in_sight else DIRECTED
        return self

    # --- what the drive is for ------------------------------------------------

    @property
    def extinguished(self):
        """Say whether expectancy has fallen far enough to give up the episode."""
        return self.expectancy < self.extinction_threshold

    @property
    def directed(self):
        """Say whether the circuit is aimed at a particular object."""
        return bool(self.object_class)

    def search_radius(self):
        """
        How wide the search should currently work, in metres.

        Falling expectancy widens it: the less the robot believes the object is
        where it was told, the more ground it has to be willing to cover. At
        full expectancy the search stays tight around the remembered place,
        which is where the object usually is.
        """
        span = self.search_radius_max - self.search_radius_min
        return self.search_radius_min + span * (1.0 - self.expectancy)

    def detection_threshold(self):
        """
        Confidence a detection needs to be acted on -- incentive salience.

        Panksepp's third attribute is that the circuit modulates sensory input,
        and this is that, as a number the perception gate actually uses: a
        strongly seeking robot accepts a weaker detection of the thing it is
        looking for. It is the reason a hungry animal sees food in a shadow.

        Bounded at both ends, because a robot that will accept anything is not
        motivated, it is broken.
        """
        span = self.detection_threshold_high - self.detection_threshold_low
        return self.detection_threshold_high - span * self.arousal

    def snapshot(self):
        """Return the state as a plain dict, for the message and for logging."""
        return {
            "object_class": self.object_class,
            "arousal": self.arousal,
            "expectancy": self.expectancy,
            "object_in_sight": self.object_in_sight,
            "seconds_since_cue": self.seconds_since_cue,
            "search_radius": self.search_radius(),
            "phase": self.phase,
        }

    def __repr__(self):
        """Show both layers and the phase, which is what a log line wants."""
        return (
            f"SeekingDrive({self.phase}, arousal={self.arousal:.2f}, "
            f"expectancy={self.expectancy:.2f}, object={self.object_class!r})"
        )
