(define (problem shaft1_task)
  (:domain assembly_domain)

  (:objects
    shaft1 - part
    start
    pickup_position_shaft1
    place_position_shaft1 - location
  )

  (:init
    (at shaft1 start)
    (gripper_empty)
  )

  (:goal
    (and
      (inserted shaft1)
      (gripper_empty)
    )
  )
)
