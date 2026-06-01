# Encoding Scheme and Initial Solution Construction for the Truck--Drone Resupply Problem

## 1. Problem Context

We consider a truck--drone resupply problem with:

- \(n\) trucks;
- \(m\) drones;
- a set of customers \(C=\{1,2,\ldots,N\}\);
- each customer \(i\in C\) has one package;
- each package has a release date \(r_i \ge 0\);
- trucks perform customer deliveries;
- drones do not directly serve customers, but resupply packages to trucks at rendezvous nodes;
- if a package is not available on the truck, the truck may either:
  - return to the depot to collect the package, or
  - receive the package from a drone at a customer node.

The goal of the encoding is to represent both the truck routes and the resupply decisions in a compact solution structure suitable for a genetic algorithm.

---

## 2. Solution Encoding

A solution is represented as a list of truck routes:

```text
Solution = [Route_1, Route_2, ..., Route_n]
```

Each route corresponds to the itinerary of one truck.

For example:

```text
Solution = [
    [0[1,2], 1, 2, 0[3,5], 3[4], 4, 5, 0],
    [0[6,7], 6, 7[8], 9, 8, 10, 0]
]
```

This means:

```text
Route_1 = [0[1,2], 1, 2, 0[3,5], 3[4], 4, 5, 0]
Route_2 = [0[6,7], 6, 7[8], 9, 8, 10, 0]
```

The index of the route in the outer list identifies the truck. For example:

```text
Solution[1] corresponds to truck 1.
Solution[2] corresponds to truck 2.
```

---

## 3. Meaning of Each Element

### 3.1 Depot Loading

```text
0[P]
```

means that the truck is at the depot and loads the set of packages \(P\).

Example:

```text
0[1,2]
```

means that the truck departs from the depot with packages 1 and 2.

Another example:

```text
0[3,5]
```

means that the truck returns to the depot and loads packages 3 and 5 before continuing its route.

---

### 3.2 Customer Visit

```text
i
```

means that the truck visits customer \(i\) and delivers package \(i\).

Example:

```text
1
```

means that the truck serves customer 1.

---

### 3.3 Drone Resupply at a Customer Node

```text
i[P]
```

means that, at customer node \(i\), the truck receives the set of packages \(P\) from a drone.

Example:

```text
3[4]
```

means that the truck receives package 4 from a drone at customer node 3.

The truck then carries package 4 and later serves customer 4.

---

## 4. Important Encoding Rules

### Rule 1. A Drone Can Resupply Multiple Packages at One Rendezvous Node

The encoding allows a drone to resupply more than one package at a customer node.

Example:

```text
Route_1 = [0[1,2], 1, 2, 0[3], 3[4,5], 4, 5, 0]
```

Here:

```text
3[4,5]
```

means that, at customer node 3, the truck receives packages 4 and 5 from a drone.

The drone does not directly serve customers 4 and 5. It only transfers the packages to the truck at node 3. The truck then delivers them later.

---

### Rule 2. A Package Does Not Need to Be Resupplied at Its Own Customer Node

A package \(j\) can be resupplied at an earlier node \(i\), where \(i\neq j\).

Example:

```text
Route_1 = [0[1,2], 1, 2, 3[5], 4, 5, 0]
```

Here, package 5 is resupplied at node 3, and customer 5 is served later.

This is feasible if package 5 is already on the truck before the truck reaches customer 5.

---

### Rule 3. A Package Must Be Available on the Truck Before Delivery

For every customer \(j\), package \(j\) must be available on the truck before the truck visits customer \(j\).

Therefore, the following route is feasible with respect to package availability:

```text
[0[1,2], 1, 2, 3[5], 4, 5, 0]
```

because package 5 is received at node 3 before customer 5 is visited.

However, the following route is infeasible:

```text
[0[1,2], 1, 5, 3[5], 4, 0]
```

because customer 5 is visited before package 5 is resupplied.

---

### Rule 4. The Last Depot Node Marks the End of a Truck Route

A depot node with packages indicates a depot loading or reloading operation:

```text
0[P]
```

A depot node without packages at the end of a route indicates that the truck returns to the depot and finishes its route:

```text
..., 5, 0
```

For example:

```text
[0[1,2], 1, 2, 0[3,5], 3[4], 4, 5, 0]
```

The first depot node:

```text
0[1,2]
```

means initial loading.

The middle depot node:

```text
0[3,5]
```

means the truck returns to the depot to load additional packages.

The final depot node:

```text
0
```

means the truck finishes its route.

---

## 5. Feasibility Conditions

For a solution to be feasible, the following conditions should be checked.

### 5.1 Customer Uniqueness

Each customer must appear exactly once as a served customer in the truck routes.

That is, each customer \(i\in C\) must appear exactly once as a customer visit.

---

### 5.2 Package Availability

For each customer \(i\), package \(i\) must be available on the truck before customer \(i\) is served.

A package may become available on the truck in one of two ways:

1. the truck loads it at the depot via `0[P]`;
2. the truck receives it from a drone via `j[P]`.

---

### 5.3 Release Date Feasibility

A package \(i\) can only be loaded or resupplied after its release date \(r_i\).

For depot loading:

```text
0[P]
```

the truck can depart from the depot with package \(i\in P\) only if:

```text
departure_time_from_depot >= r_i
```

For drone resupply:

```text
j[P]
```

the drone can depart from the depot with package \(i\in P\) only if:

```text
drone_departure_time >= r_i
```

If multiple packages are resupplied in the same drone trip, then:

```text
drone_departure_time >= max{r_i | i in P}
```

---

### 5.4 Truck Capacity Feasibility

At any segment of a truck route, the total load on the truck must not exceed truck capacity \(Q_T\).

For all packages currently onboard:

```text
sum(q_i) <= Q_T
```

---

### 5.5 Drone Capacity Feasibility

For each drone resupply operation:

```text
j[P]
```

the total load carried by the drone must not exceed drone capacity \(Q_D\):

```text
sum(q_i for i in P) <= Q_D
```

---

### 5.6 Drone Timing Feasibility

For a drone resupply operation at node \(j\), the drone must arrive no later than the time the truck leaves node \(j\):

```text
arrival_time_drone(j) <= departure_time_truck(j)
```

If the drone arrives later, either:

- the truck must wait for the drone, or
- the resupply operation is infeasible.

---

### 5.7 Drone Endurance Feasibility

For each drone trip, the total flying time or energy consumption must satisfy the drone endurance constraint.

For example:

```text
drone_flight_time <= L
```

or, under an energy-based model:

```text
drone_energy_consumption <= E_max
```

---

## 6. Initial Solution Construction

The initial solution construction follows a layered approach:

```text
Step 1. Generate a customer sequence.
Step 2. Split customers among trucks.
Step 3. Construct each truck route.
Step 4. Insert depot returns and drone resupply operations.
Step 5. Repair infeasible components.
```

This is preferable to generating the full encoded route randomly, because direct random generation often produces infeasible individuals.

---

## 7. Initial Population Construction

```text
Algorithm INIT_POPULATION

Input:
    C        : set of customers
    K        : set of trucks, |K| = n
    D        : set of drones, |D| = m
    r_i      : release date of package i
    q_i      : demand or weight of package i
    Q_T      : truck capacity
    Q_D      : drone capacity
    P_size   : population size

Output:
    Population

Begin
    Population <- empty set

    for p = 1 to P_size do

        strategy <- randomly select one construction strategy
                    from {Random, ReleaseDate, NearestNeighbor, Mixed}

        pi <- GENERATE_CUSTOMER_SEQUENCE(C, strategy)

        BaseRoutes <- SPLIT_CUSTOMERS_TO_TRUCKS(pi, K)

        Solution <- empty list
        DroneSchedule <- empty schedule

        for each truck k in K do

            Route_k <- INIT_TRUCK_ROUTE(BaseRoutes[k], k, DroneSchedule)

            append Route_k to Solution

        end for

        Solution <- REPAIR_SOLUTION(Solution, DroneSchedule)

        add Solution to Population

    end for

    return Population
End
```

---

## 8. Generating a Customer Sequence

```text
Algorithm GENERATE_CUSTOMER_SEQUENCE

Input:
    C        : set of customers
    strategy : selected construction strategy

Output:
    pi       : ordered sequence of customers

Begin
    if strategy = Random then

        pi <- random permutation of C

    else if strategy = ReleaseDate then

        pi <- sort customers in C by increasing release date

    else if strategy = NearestNeighbor then

        pi <- empty sequence
        current <- depot
        U <- C

        while U is not empty do

            choose customer j in U with small score:

                score(j) = alpha * distance(current, j)
                         + beta  * release_date(j)
                         + gamma * random_noise

            append j to pi
            current <- j
            remove j from U

        end while

    else if strategy = Mixed then

        pi <- randomized combination of distance priority and release-date priority

    end if

    return pi
End
```

---

## 9. Splitting Customers Among Trucks

```text
Algorithm SPLIT_CUSTOMERS_TO_TRUCKS

Input:
    pi : ordered customer sequence
    K  : set of trucks

Output:
    BaseRoutes[k] for each truck k

Begin
    for each truck k in K do

        BaseRoutes[k] <- empty sequence
        EstimatedTime[k] <- 0

    end for

    for each customer i in pi do

        choose truck k* with the smallest EstimatedTime[k]

        append i to BaseRoutes[k*]

        update EstimatedTime[k*] based on the estimated completion time
        of BaseRoutes[k*]

    end for

    return BaseRoutes
End
```

Example:

```text
pi = [1, 2, 3, 4, 5, 6, 7, 9, 8, 10]
```

After splitting, we may obtain:

```text
BaseRoutes[1] = [1, 2, 3, 4, 5]
BaseRoutes[2] = [6, 7, 9, 8, 10]
```

---

## 10. Initial Construction of One Truck Route

```text
Algorithm INIT_TRUCK_ROUTE

Input:
    R             : customer sequence assigned to truck k
    k             : current truck
    DroneSchedule : current drone schedule

Output:
    EncodedRoute_k

Begin
    EncodedRoute <- empty route

    current_node <- depot
    current_time <- 0
    onboard <- empty set
    pos <- 1

    P0 <- SELECT_DEPOT_LOAD(R, pos, current_time, onboard)

    append 0[P0] to EncodedRoute

    onboard <- onboard union P0

    while pos <= length(R) do

        j <- R[pos]

        if j is not in onboard then

            option_drone <- FIND_DRONE_RESUPPLY_SET(
                                R,
                                pos,
                                current_node,
                                current_time,
                                DroneSchedule
                            )

            option_return <- FIND_DEPOT_RETURN(
                                package = j,
                                current_node,
                                current_time,
                                R,
                                pos
                            )

            selected_option <- SELECT_RESUPPLY_OPTION(option_drone, option_return)

            if selected_option = Drone then

                P_drone <- option_drone.package_set

                add P_drone into the bracket of current_node in EncodedRoute

                onboard <- onboard union P_drone

                update DroneSchedule

                if drone arrival time > current_time then
                    current_time <- drone arrival time
                end if

            else if selected_option = ReturnDepot then

                current_time <- current_time + travel_time(current_node, depot)

                P <- SELECT_DEPOT_LOAD(R, pos, current_time, onboard)

                if j is not in P then
                    current_time <- max(current_time, r_j)
                    add j to P
                end if

                append 0[P] to EncodedRoute

                onboard <- onboard union P

                current_node <- depot

            end if

        end if

        current_time <- current_time + travel_time(current_node, j)

        append j to EncodedRoute

        serve customer j

        remove j from onboard

        current_node <- j

        P_future <- SELECT_OPTIONAL_DRONE_RESUPPLY_SET(
                        R,
                        pos + 1,
                        current_node,
                        current_time,
                        DroneSchedule
                    )

        if P_future is not empty then

            add P_future into the bracket of node j in EncodedRoute

            onboard <- onboard union P_future

            update DroneSchedule

            if drone arrival time > current_time then
                current_time <- drone arrival time
            end if

        end if

        pos <- pos + 1

    end while

    append 0 to EncodedRoute

    return EncodedRoute
End
```

---

## 11. Selecting Packages Loaded at the Depot

```text
Algorithm SELECT_DEPOT_LOAD

Input:
    R            : customer sequence of the truck
    pos          : position of the next customer to be served
    current_time : current time at the depot
    onboard      : set of packages already on the truck

Output:
    P            : set of packages loaded at the depot

Begin
    P <- empty set
    remaining_capacity <- Q_T - total_weight(onboard)

    for h = pos to length(R) do

        i <- R[h]

        if i is not in onboard
           and r_i <= current_time
           and q_i <= remaining_capacity then

            add i to P

            remaining_capacity <- remaining_capacity - q_i

        end if

        if remaining_capacity = 0 then
            break
        end if

    end for

    return P
End
```

This rule loads available packages of upcoming customers, subject to truck capacity.

---

## 12. Selecting a Set of Packages for Drone Resupply

This procedure allows the drone to resupply multiple packages at the same rendezvous node.

```text
Algorithm FIND_DRONE_RESUPPLY_SET

Input:
    R             : customer sequence of the current truck
    pos           : current position in the route
    current_node  : current rendezvous node
    current_time  : truck arrival time at current_node
    DroneSchedule : current drone schedule

Output:
    option_drone  : feasible package set and its cost, or infeasible

Begin
    if current_node = depot then
        return infeasible
    end if

    P_drone <- empty set
    remaining_capacity <- Q_D

    Candidate <- customers from position pos to the end of R
                 whose packages are not yet onboard

    sort Candidate by route order or by increasing release date

    for each customer i in Candidate do

        if q_i <= remaining_capacity then

            earliest_drone_departure <- r_i

            drone_arrival_time <- earliest_drone_departure
                                + travel_time_drone(depot, current_node)

            if drone_arrival_time <= current_time + allowed_wait
               and a drone is available
               and drone endurance is satisfied then

                add i to P_drone

                remaining_capacity <- remaining_capacity - q_i

            end if

        end if

    end for

    if P_drone is empty then
        return infeasible
    end if

    cost_drone <- estimated waiting time or additional synchronization cost

    return feasible option with P_drone and cost_drone
End
```

---

## 13. Finding a Truck Return-to-Depot Option

```text
Algorithm FIND_DEPOT_RETURN

Input:
    package j
    current_node
    current_time
    R
    pos

Output:
    option_return

Begin
    arrival_depot_time <- current_time + travel_time(current_node, depot)

    waiting_time <- max(0, r_j - arrival_depot_time)

    departure_depot_time <- arrival_depot_time + waiting_time

    return_to_customer_time <- travel_time(depot, j)

    cost_return <- travel_time(current_node, depot)
                 + waiting_time
                 + return_to_customer_time

    return feasible option with cost_return
End
```

The truck return option is always feasible if the truck can wait at the depot until the package is released.

---

## 14. Selecting Between Drone Resupply and Depot Return

```text
Algorithm SELECT_RESUPPLY_OPTION

Input:
    option_drone
    option_return

Output:
    selected option

Begin
    if option_drone is infeasible then

        return ReturnDepot

    else if option_return is infeasible then

        return Drone

    else if cost_drone < cost_return then

        choose Drone with probability 0.7
        choose ReturnDepot with probability 0.3

    else

        choose ReturnDepot with probability 0.7
        choose Drone with probability 0.3

    end if
End
```

This biased random rule provides both quality and diversity in the initial population.

---

## 15. Optional Drone Resupply After Serving a Customer

After serving a customer \(i\), the algorithm may proactively resupply packages for future customers at node \(i\).

```text
Algorithm SELECT_OPTIONAL_DRONE_RESUPPLY_SET

Input:
    R
    next_pos
    current_node
    current_time
    DroneSchedule

Output:
    P_future

Begin
    P_future <- empty set

    With a given probability p_resupply, attempt drone resupply.

    If resupply is attempted, call FIND_DRONE_RESUPPLY_SET
    using current_node as the rendezvous node.

    If a feasible package set is found, return that package set.

    Otherwise, return empty set.
End
```

This step helps generate routes such as:

```text
[0[1,2], 1, 2, 3[4,5], 4, 5, 0]
```

where packages 4 and 5 are supplied by a drone at node 3 before they are delivered by the truck.

---

## 16. Repair Procedure

After the initial solution is constructed, a repair procedure is applied.

```text
Algorithm REPAIR_SOLUTION

Input:
    Solution
    DroneSchedule

Output:
    Repaired Solution

Begin
    Repair 1:
        Ensure each customer appears exactly once in the truck routes.

    Repair 2:
        Ensure each customer i is served only after package i is available
        on the corresponding truck.

    Repair 3:
        Ensure every depot loading operation 0[P] satisfies release dates.

    Repair 4:
        Ensure truck capacity is not violated on any route segment.

    Repair 5:
        Ensure every drone resupply operation satisfies:
            - release date;
            - drone capacity;
            - drone endurance;
            - drone availability;
            - timing synchronization with the truck.

    Repair 6:
        If a drone resupply operation is infeasible:
            try moving it to another earlier rendezvous node;
            otherwise replace it by a depot return 0[P].

    return repaired Solution
End
```

---

## 17. Example of an Initial Solution

An initialized solution may have the following form:

```text
Solution = [
    [0[1,2], 1, 2, 0[3], 3[4,5], 4, 5, 0],
    [0[6,7], 6, 7[8,10], 9, 8, 10, 0]
]
```

Interpretation:

```text
Route_1:
    Truck 1 departs from the depot with packages 1 and 2.
    It serves customers 1 and 2.
    It returns to the depot and loads package 3.
    At customer 3, a drone resupplies packages 4 and 5.
    The truck then serves customers 4 and 5 and returns to the depot.

Route_2:
    Truck 2 departs from the depot with packages 6 and 7.
    It serves customers 6 and 7.
    At customer 7, a drone resupplies packages 8 and 10.
    The truck then serves customers 9, 8, and 10 and returns to the depot.
```

---

## 18. Summary

The proposed encoding is:

```text
Solution = [Route_1, Route_2, ..., Route_n]
```

where each route is a sequence of depot visits, customer visits, and drone resupply annotations.

The key features are:

- multiple trucks are represented by multiple routes;
- depot loading is represented by `0[P]`;
- drone resupply is represented by `i[P]`;
- a drone can resupply multiple packages at one rendezvous node;
- a package can be resupplied at a node different from its customer node;
- a package only needs to be available on the truck before its corresponding customer is served;
- the initialization procedure constructs feasible or near-feasible individuals by combining truck route generation, depot loading, drone resupply selection, and repair.

This encoding is suitable for a genetic algorithm because it clearly separates:

```text
1. customer assignment to trucks;
2. customer visiting order;
3. depot return decisions;
4. drone resupply decisions.
```
