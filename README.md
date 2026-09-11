# Warehouse Robot Coordination System

A Python-based decentralized multi-robot warehouse coordination system designed to reduce dependency on a central controller.

The system combines multi-hop mesh communication, task bidding, path planning, time-based path reservation, collision detection, conflict resolution, and deadlock recovery to coordinate multiple warehouse robots.

## Key Features

- **Decentralized robot coordination**
  - Robots participate in a distributed communication network rather than relying entirely on a single central controller.

- **Multi-hop mesh communication**
  - Robots dynamically maintain communication links with nearby robots.
  - State information can be propagated through the active mesh.

- **Task bidding and assignment**
  - Available robots evaluate tasks using factors such as battery level, distance, availability, and task priority.
  - The most suitable eligible robot is selected for each task.

- **Path reservation**
  - Robots reserve future cells/edges before movement to reduce conflicts between robots.

- **Collision avoidance**
  - The coordinator checks vertex conflicts, target conflicts, swap/edge conflicts, and physical occupancy conflicts before allowing movement.

- **Deadlock recovery**
  - The system monitors lack of progress and attempts recovery when robots become deadlocked.

- **Multi-robot simulation**
  - Designed and tested with multiple robots operating simultaneously in a warehouse environment.

## System Architecture

```text
                    ┌─────────────────────────┐
                    │       Task Generator    │
                    └────────────┬────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────┐
                    │     Task Bidding        │
                    │  Priority + Distance +  │
                    │  Battery + Availability │
                    └────────────┬────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────┐
                    │   Robot Coordinator     │
                    └────────────┬────────────┘
                                 │
              ┌──────────────────┼──────────────────┐
              ▼                  ▼                  ▼
       ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
       │ Path Planner │   │ Reservation │   │   Conflict  │
       │             │   │   Manager   │   │   Resolver  │
       └─────────────┘   └─────────────┘   └─────────────┘
              │                  │                  │
              └──────────────────┼──────────────────┘
                                 ▼
                    ┌─────────────────────────┐
                    │     Robot Movement      │
                    └────────────┬────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────┐
                    │    Mesh Communication   │
                    │      Between Robots     │
                    └─────────────────────────┘
## Demo

### Multi-Robot Warehouse Simulation

The system coordinates multiple robots in a warehouse environment using decentralized communication, task assignment, path planning, reservation, and conflict resolution.

![Warehouse Robot Coordination Demo](screenshots/Screenshot%202026-09-11%20213701.png)
                    
                    
