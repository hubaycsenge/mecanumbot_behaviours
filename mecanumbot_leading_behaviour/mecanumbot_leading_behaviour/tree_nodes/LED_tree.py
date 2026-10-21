"""
LED leading behaviour: same route, signalled with light patterns.

This is the non-animal comparison condition, so the robot never gestures with
its head: there is no look back, no alternation, no gesture script. The one
thing the head does is perceptual. While the robot is **looking for the
person** -- the opening approach, the recovery search, the turn onto them, the
scans at the target -- it is tilted to the height of the body the LiDAR expects
(the body gaze, started below as a load hook; `LevelHead` parks it again as soon
as the person is found and faced, before any signal is shown), because one fixed tilt shows the
camera a standing person's knees up close and misses somebody sitting low. It
does not nod: the look down onto a low face that the dog-inspired tree's search
makes is switched off here (`look_down=False`), so the tilt compensation is the
only thing that moves the neck. For
the drive to the target and the signalling there the head is parked level
(`LevelHead`, and the turn onto the target levels it too).
`body_gaze_enabled: false` in the constants file gives the fixed head back.

The lights are on for the whole drive, not only before and after it: they flow
the way the robot is going and fill with a second colour as it gets there
(`LEDLeadSignal`). Asking for attention uses all four panels; the thank-you for
the ball is shown on the half of the robot that faces the person.

At the end of the route the robot turns to the target and plays the green
`indicate_target` signal. What the person gets next depends on where they are.
Somebody within `Dog_following_max_threshold` of the robot -- the dog-inspired
tree's "is the human with the robot", set on the web GUI's behaviour page -- can
read the panels, so the
robot turns to them and blinks the ones that point at the target in yellow
(`LEDTargetDirection`). Somebody further off, or out of view, is a person the
robot goes to -- searching for them first if it has to -- and asks for
attention. Either way it then turns back to the target and plays the green
signal again.
"""

import py_trees

from mecanumbot_leading_behaviour.behaviours.LED_behaviours import (
    LEAD_KEYS,
    CheckSubjectNearRobot,
    LEDBehaviourSequence,
    LEDLeadSignal,
    LEDTargetDirection,
    LevelHead,
)
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import (
    LED_SCRIPTS,
    ConfiguredTimer,
    ConstantParamsToBlackboard,
)
from mecanumbot_leading_behaviour.behaviours.route_behaviours import (
    Approach,
    CheckRobotAtLastCheckpoint,
    CheckRobotHasBall,
    CheckSubjectTargetSuccess,
    FindPeople,
    TurnToward,
)
from mecanumbot_movement_behaviours.defaults import start_body_gaze
from mecanumbot_movement_behaviours.turning import SHORTEST
from mecanumbot_movement_behaviours.targets import (
    LAST_CHECKPOINT,
    SUBJECT,
    TARGET,
)
from mecanumbot_leading_behaviour.behaviours.start_gate import WaitForTrialStart
from mecanumbot_leading_behaviour.tree_nodes.tree_common import (
    create_recover_lost_sequence,
    resolve_yaml_path,
    run_tree,
)

TREE_NAME = "LED_tree"
DEFAULT_YAML_FILENAME = "Eto_behaviour_setting_constants.yaml"


def get_yaml_path():
    return resolve_yaml_path(TREE_NAME, DEFAULT_YAML_FILENAME)


def create_root(yaml_path=None):
    if yaml_path is None:
        yaml_path = get_yaml_path()

    # --- reach the human: directly, or search the route for them first ------
    recover_then_approach = py_trees.composites.Sequence(
        name="RecoverThenApproach", memory=True
    )
    recover_then_approach.add_children(
        [
            create_recover_lost_sequence(ID="Init", look_down=False),
            Approach(
                name="ApproachSubjectRecov", target_type=SUBJECT, mode="fixed_distance"
            ),
        ]
    )

    seek_or_find = py_trees.composites.Selector("SeekOrFind", memory=True)
    seek_or_find.add_children(
        [
            Approach(name="ApproachSubject", target_type=SUBJECT),
            recover_then_approach,
        ]
    )

    # --- keep signalling the target while the human stays near it ------------
    show_while_close = py_trees.composites.Sequence(
        name="ShowWhileSubjectClose", memory=True
    )
    show_while_close.add_children(
        [
            FindPeople(name="FindPersonClose"),
            CheckSubjectTargetSuccess(name="CheckSubjectNearTarget"),
            # The person is at the target, so near: they are shown where it is
            # rather than asked for attention.
            LevelHead(name="ParkHeadToPointClose"),
            LEDTargetDirection(name="LDirectionClose", target_type=TARGET),
            TurnToward(name="TurnTowardTarget", target_type=TARGET, head=None),
            LEDBehaviourSequence("LNear", "indicate_close_target"),
        ]
    )

    # --- the human handed the ball over -------------------------------------
    # Turn straight to where the person was last seen and thank them. Only if
    # nobody has been seen does the robot spin to find somebody first. Until
    # 2026-10-21 it always spun first (`FindPeople`), and the check was only
    # reached once the whole end-of-route signalling had played out, so the
    # thank-you came late and after a search for a person standing right there.
    face_giver = py_trees.composites.Selector(name="FaceBallGiver", memory=True)
    face_giver.add_children(
        [
            TurnToward(
                name="TurnTowardBallGiver",
                target_type=SUBJECT,
                head=None,
                direction=SHORTEST,
            ),
            FindPeople(name="FindPersonBallReaction"),
        ]
    )
    ball_reaction = py_trees.composites.Sequence(name="BallReactionSeq", memory=True)
    ball_reaction.add_children(
        [
            CheckRobotHasBall(name="CheckIfHasBall"),
            face_giver,
            LEDBehaviourSequence("LThank", "thank", addressed=True),
            # Paces the thank-you while the robot holds on to the ball, as in
            # the dog tree; without it the loop played it back to back.
            ConfiguredTimer(name="ThankDelayTimer", key="thank_delay"),
        ]
    )

    # --- lead: the lights run for the whole drive ----------------------------
    # The drive is what ends the parallel; the signal beside it never finishes
    # by itself, which is why the policy names the drive rather than "one".
    drive = Approach(name="ApproachTarget", target_type=LAST_CHECKPOINT)
    lead = py_trees.composites.Parallel(
        name="LeadWithLights",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[drive], synchronise=False
        ),
    )
    lead.add_children(
        [LEDLeadSignal(name="LLead", target_type=LAST_CHECKPOINT), drive]
    )

    # --- arrived: address the person, by how far away they are ---------------
    # Near: turn to them and point at the target with the lights.
    point_out = py_trees.composites.Sequence(name="PointOutTarget", memory=True)
    point_out.add_children(
        [
            CheckSubjectNearRobot(name="CheckSubjectNearRobot"),
            TurnToward(name="TurnTowardSubjectAtEnd", target_type=SUBJECT, head=None),
            LevelHead(name="ParkHeadToPoint"),
            LEDTargetDirection(name="LDirection", target_type=TARGET),
        ]
    )

    # Far, or not in view: go to them the way the opening does -- directly, or
    # after searching the route -- and ask for attention there.
    search_then_approach = py_trees.composites.Sequence(
        name="SearchThenApproachAtEnd", memory=True
    )
    search_then_approach.add_children(
        [
            create_recover_lost_sequence(ID="End", look_down=False),
            Approach(
                name="ApproachSubjectEndRecov",
                target_type=SUBJECT,
                mode="fixed_distance",
            ),
        ]
    )
    reach_person = py_trees.composites.Selector("ReachPersonAtEnd", memory=True)
    reach_person.add_children(
        [
            Approach(name="ApproachSubjectAtEnd", target_type=SUBJECT),
            search_then_approach,
        ]
    )
    fetch_attention = py_trees.composites.Sequence(name="GoAndCatchAttention", memory=True)
    fetch_attention.add_children(
        [
            reach_person,
            TurnToward(name="TurnTowardSubjectFound", target_type=SUBJECT, head=None),
            LevelHead(name="ParkHeadToCatch"),
            LEDBehaviourSequence("LCatch", "catch_attention"),
        ]
    )

    address_person = py_trees.composites.Selector("AddressPersonAtEnd", memory=True)
    address_person.add_children([point_out, fetch_attention])

    # --- after arriving: show the target, unless the ball has been handed over
    # The signalling at the end of the route. Its last step repeats while the
    # human stays at the target; when they are not, it fails, and so does the
    # root, which finds the human, asks for attention and leads again.
    show_at_end = py_trees.composites.Sequence(name="ShowAtEndSeq", memory=True)
    show_at_end.add_children(
        [
            # The turns onto the target go the short way round. The library's
            # default for a place unwinds the last search turn, which here was
            # most of a circle (316 degrees in the run of 2026-10-05).
            TurnToward(
                name="TurnTowardTargetAtEnd",
                target_type=TARGET,
                head=None,
                direction=SHORTEST,
            ),
            LEDBehaviourSequence("LShow", "indicate_target"),
            address_person,
            TurnToward(
                name="TurnBackTowardTarget",
                target_type=TARGET,
                head=None,
                direction=SHORTEST,
            ),
            LEDBehaviourSequence("LShowAgain", "indicate_target"),
            py_trees.decorators.Repeat(
                name="ShowWhileCloseLoop", child=show_while_close, num_success=-1
            ),
        ]
    )

    # A selector without memory asks for the ball first on every tick, so the
    # moment it is handed over the signal or turn under way is stopped and the
    # robot turns to the person and thanks them. With memory, the ball was only
    # looked at between rounds of signalling.
    after_arrival = py_trees.composites.Selector("BallFirstAfterArrival", memory=False)
    after_arrival.add_children(
        [
            py_trees.decorators.Repeat(
                name="BallReactionRepeat", child=ball_reaction, num_success=-1
            ),
            show_at_end,
        ]
    )

    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [
            ConstantParamsToBlackboard(
                name="LoadConstantParams",
                yaml_path=yaml_path,
                scripts=LED_SCRIPTS,
                required=tuple(LEAD_KEYS.values()),
                on_loaded=(start_body_gaze,),
            ),
            # Hold still until somebody comes in: a person is tracked, or the
            # LiDAR sees something new in the room (start_gate.py, trial_start.py).
            WaitForTrialStart(name="WaitForTrialStart"),
            seek_or_find,
            TurnToward(name="TurnTowardSubject", target_type=SUBJECT, head=None),
            # The person is found and faced: from here to the next time the
            # robot looks for them the head is parked, signals included.
            LevelHead(name="ParkHeadToLead"),
            LEDBehaviourSequence("LCatchO", "catch_attention"),
            # Drive to the last checkpoint of the route, then face the target itself
            # to signal it -- the robot stops short of where the human should end up.
            lead,
            # Nothing asks for the ball from anywhere but the end of the route.
            # The drive only succeeds there, so this holds already; it is here
            # so that it goes on holding. A robot that is not there fails the
            # root, which finds the human and leads again. The route index is
            # left out because this tree never walks it.
            CheckRobotAtLastCheckpoint(name="CheckRobotAtRouteEnd", by_index=False),
            # From here on the ball comes first: `after_arrival` re-asks for it
            # on every tick and cuts into whatever signal or turn is running.
            after_arrival,
        ]
    )
    return root


def main(args=None):
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)


if __name__ == "__main__":
    main()
