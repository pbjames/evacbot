# Autonomous safety evaluation & rescue for subterranean confined spaces

## Evaluation

### Considerations

### Modelling the environment

## Goals

- Reproducible mappings of the environment can be made which sufficiently communicates
  the risks to human life in an enclosed space.
- Our robot should work correctly in natural, urban and industrial environments.
- Navigation is efficient, detailed and doesn't put the robot in harm's way
- Our robot still works in previously unseen environments

## Approach

### Pathing

Using the A\* algorithm in combination with appropriate movement methods and hazard
identification & localisation will calculate optimal traversal of the space.

This will be done by constructing a node graph of space using mappings and assigning
relative weights based on actual calculated distance between nodes. Nodes near hazardous
objects or sources will have a higher weight. Nodes with the most empty space and no
local hazards will be the lowest weighted.

The algorithm will continually update the node graph and traverse safely in order to be
reactive to dynamic situations.

### SLAM

### Hazard Identification
