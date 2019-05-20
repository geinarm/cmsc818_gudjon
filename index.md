---
layout: default
---

# Task and Motion Planning for Robotic Arm

The initial goal oft this project was to identify a method to integrate task and motion planning (TAMP), and to apply the planner to a robotic arm performing pick and place tasks in a 2D workspace. The TAMP planner should plan a sequence of task such that it consider the geometric constraints of performing the task. A symbolic task planner that is not aware of the physical constraints of the workspace may plan a sequence of tasks that logically achieve the goal but are physically infeasible.

TAMP is a research topic that is becoming more important as robotic get better and we want them to solve increasingly complicated tasks. This topic has been studied for many years and many ideas have been proposed, but to my knowledge no single method has gained wide popularity. This is why I wanted to study this topic.

When I started this project I laid out a plan to build a simple test bead so I could experiment with different ideas and planner implementations. But when I got started I found that my simple 2D robotic arm actually contained a lot of hidden complexity. I got sucked down the rabbit whole of tweaking and improving the simulation environment instead of researching TAMP. About half-way through the project time frame I decided that TAMP would be a secondary goal and I would focus on building the best test environment I could. Maybe the next time someone gets the same idea I got, he or she can use my work as a starting point. I intent to clean up my code, document the API and share this implementation publicly.

## Result

<iframe width="560" height="315" src="https://www.youtube.com/embed/Z86i3oZe9O0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
