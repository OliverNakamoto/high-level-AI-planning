(define (domain mission-planning-ttk4192)
  (:requirements :typing :durative-actions)
  (:types
    robot waypoint valve pump charger route
  )

  (:predicates
    (at              ?r  - robot     ?w  - waypoint)
    (connects        ?d  - route     ?w1 - waypoint ?w2 - waypoint)
    (valve-at        ?v  - valve     ?w  - waypoint)
    (pump-at         ?p  - pump      ?w  - waypoint)
    (charger-at      ?c  - charger   ?w  - waypoint)
    (valve-checked   ?v  - valve)
    (pump-photographed ?p - pump)
    (charged         ?r  - robot)
  )

  (:functions
    (speed         ?r - robot)
    (route-length  ?d - route)
  )

  ;;——— move along a route ———
  (:durative-action move
    :parameters (?r - robot ?from ?to - waypoint ?d - route)
    :duration (= ?duration (/ (route-length ?d) (speed ?r)))
    :condition (and
      (at start    (at ?r ?from))
      (at start    (connects ?d ?from ?to))
    )
    :effect (and
      (at start    (not (at ?r ?from)))
      (at end      (at   ?r ?to))
    )
  )

  ;;——— take a picture of a pump ———
  (:durative-action take-picture
    :parameters (?r - robot ?p - pump ?w - waypoint)
    :duration (= ?duration 3)    ;; seconds, adjust as needed
    :condition (and
      (at start (at ?r ?w))
      (at start (pump-at ?p ?w))
    )
    :effect (and
      (at end   (pump-photographed ?p))
    )
  )

  ;;——— manipulate/inspect a valve ———
  (:durative-action manipulate-valve
    :parameters (?r - robot ?v - valve ?w - waypoint)
    :duration (= ?duration 5)    ;; seconds, adjust as needed
    :condition (and
      (at start (at ?r ?w))
      (at start (valve-at ?v ?w))
    )
    :effect (and
      (at end   (valve-checked ?v))
    )
  )

  ;;——— charge robot’s battery ———
  (:durative-action charge
    :parameters (?r - robot ?c - charger ?w - waypoint)
    :duration (= ?duration 10)   ;; seconds, adjust as needed
    :condition (and
      (at start (at ?r ?w))
      (at start (charger-at ?c ?w))
    )
    :effect (and
      (at end   (charged ?r))
    )
  )
)
