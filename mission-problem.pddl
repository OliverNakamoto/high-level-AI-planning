(define (problem inspection-round)
  (:domain mission-planning-ttk4192)

  (:objects
    turtlebot0                           - robot
    valve0 valve1                        - valve
    pump0  pump1                         - pump
    charger0 charger1 charger2          - charger
    waypoint0 waypoint1 waypoint2 waypoint3 waypoint4 waypoint5 waypoint6  - waypoint
    d01 d10 d23 d32 d26 d62 d14 d41 d46 d64 d35 d53  - route
  )

  (:init
    (= (speed turtlebot0) 0.18)

    ;; bidirectional waypoint-waypoint lengths
    (= (route-length d01)  9.3) (= (route-length d10)  9.3)
    (= (route-length d23)  8.6) (= (route-length d32)  8.6)
    (= (route-length d26)  5.2) (= (route-length d62)  5.2)
    (= (route-length d14) 15.0) (= (route-length d41) 15.0)
    (= (route-length d46)  9.7) (= (route-length d64)  9.7)
    (= (route-length d35) 11.4) (= (route-length d53) 11.4)

    ;; connections (both directions)
    (connects d01 waypoint0 waypoint1) (connects d10 waypoint1 waypoint0)
    (connects d23 waypoint2 waypoint3) (connects d32 waypoint3 waypoint2)
    (connects d26 waypoint2 waypoint6) (connects d62 waypoint6 waypoint2)
    (connects d14 waypoint1 waypoint4) (connects d41 waypoint4 waypoint1)
    (connects d46 waypoint4 waypoint6) (connects d64 waypoint6 waypoint4)
    (connects d35 waypoint3 waypoint5) (connects d53 waypoint5 waypoint3)

    ;; starting location
    (at turtlebot0 waypoint4)

    ;; where the objects sit
    (valve-at valve0 waypoint1) (valve-at valve1 waypoint2)
    (pump-at  pump0  waypoint5) (pump-at  pump1  waypoint6)
    (charger-at charger0 waypoint0) (charger-at charger1 waypoint4) (charger-at charger2 waypoint3)
  )

  (:goal (and
    (valve-checked     valve0)
    (valve-checked     valve1)
    (pump-photographed pump0)
    (pump-photographed pump1)
    (at turtlebot0     waypoint0)
  ))
  (:metric minimize (total-time))
)

