# Flocking Algorithm

This project is my C++ implementation of a flocking simulation, with graphical rendering provided by [raylib](https://www.raylib.com/).

It is my second attempt at implementing the flocking problem. The main goal is not only to simulate boid behavior, but also to compare the performance of three different neighbor-search methods:

* Brute-force search
* Spatial hash table
* Quadtree

The brute-force method acts as a non-optimized baseline, while the spatial hash table and quadtree are optimized approaches designed to improve performance when simulating large numbers of boids.

## Features

* Flocking based on separation, alignment, and cohesion
* Three neighbor-search implementations
* Configurable simulation parameters
* Debug visualization
* Performance measurements and CSV reports
* Experimental K-means clustering
* Mouse interaction with the flock
* Support for thousands of boids

The simulation provides parameters for the number of boids, interaction ranges, behavior strengths, maximum velocity, maximum force, field of view, spatial cell size, quadtree capacity, and maximum neighbor count.

## Demo

![Boids](media/boids.gif)

## Running the simulation

The program accepts command-line arguments that can be used to configure the simulation.

Example:

```bash
./FlockingAlgorithm -boids 5000 -method hash
```

Available neighbor-search methods:

```text
hash   Spatial hash table
qtree  Quadtree
brute  Brute-force search
```

Example with debug mode:

```bash
./FlockingAlgorithm -boids 1000 -method qtree -debug
```

The project supports arguments for window size, boid count, algorithm selection, debug mode, K-means clustering, flocking ranges and strengths, velocity limits, field of view, hash-table cell size, quadtree capacity, and maximum neighbor count.

## Debug mode

Debug mode displays additional information about the simulation.

Depending on the selected method, it can visualize:

* Spatial hash-table cells
* Quadtree partitions
* The search area of a selected boid
* Detected neighboring boids
* Velocity direction
* Separation and alignment ranges

The first boid is highlighted to make its local behavior easier to inspect.

## Experimental K-means clustering

K-means clustering can be enabled with:

```bash
./FlockingAlgorithm -boids 1000 -method hash -k_mean
```

When enabled, the simulation periodically divides the boids into distinct clusters. Each cluster is displayed using a different color, making it easier to observe how groups form and move throughout the simulation.

The number of clusters and the maximum number of K-means iterations can also be configured:

```bash
./FlockingAlgorithm -k_mean -k_mean_clusters 3 -k_mean_max_iter 3
```

This feature is computationally expensive, especially with large numbers of boids. It is not recommended for normal performance testing or high-boid-count simulations. It is included mainly as an experimental curiosity and as another way to visualize the structure of the flock.

The clustering operation is executed periodically and supports the brute-force, spatial hash-table, and quadtree methods.

## Performance reports

The simulation measures several parts of the algorithm, including:

* Spatial structure construction time
* Neighbor retrieval time
* Neighbor filtering time

Performance results are periodically recorded in CSV files. The output filename contains the selected method and number of boids, making it easier to compare different configurations and implementations.

For meaningful comparisons, K-means clustering should remain disabled because its additional computational cost can significantly affect the results.

## Mouse controls

* Hold the left mouse button to attract nearby boids.
* Hold the right mouse button to repel nearby boids.

## Technologies

* C++
* raylib
* raymath
* CMake

## Purpose

This project is primarily an experimental playground for studying flocking behavior, spatial data structures, optimization techniques, clustering, and real-time simulation performance.

It was designed to make it easy to compare different approaches, inspect their behavior visually, and test how individual parameters affect the flock.
