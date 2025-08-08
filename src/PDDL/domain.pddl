(define (domain assembly_domain)
  (:requirements :strips :typing)
  (:types part location)

  (:predicates
    (at ?p - part ?l - location)
    (grasped ?p - part)
    (gripper_empty)
    (inserted ?p - part)
  )

  ;; 1) 去 pickup：现在是三参
  (:action goto_pickup
    :parameters (?p - part   ?from - location   ?to - location)
    :precondition (and
      (at ?p ?from)
      (gripper_empty)
    )
    :effect (and
      (not (at ?p ?from))
      (at ?p ?to)
    )
  )

  ;; 2) 抓
  (:action grasp
    :parameters (?p - part)
    :precondition (and
      (at ?p pickup_position_shaft1)
      (gripper_empty)
    )
    :effect (and
      (grasped ?p)
      (not (gripper_empty))
    )
  )

  ;; 3) 去 place：也三参
  (:action goto_place
    :parameters (?p - part   ?from - location   ?to - location)
    :precondition (and
      (at ?p ?from)
      (grasped ?p)
    )
    :effect (and
      (not (at ?p ?from))
      (at ?p ?to)
    )
  )

  ;; 4) 插
  (:action insertion
    :parameters (?p - part)
    :precondition (and
      (at ?p place_position_shaft1)
      (grasped ?p)
    )
    :effect (inserted ?p)
  )

  ;; 5) 放
  (:action release_gripper
    :parameters (?p - part)
    :precondition (inserted ?p)
    :effect (and
      (not (grasped ?p))
      (gripper_empty)
    )
  )
)
