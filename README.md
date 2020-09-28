
Let's Build a Raytracer, From Scratch
=====================================
**Raytracing** is the new hotness in gaming — brillant reflections, complex refractions and high-fidelity shadows enhance the visual experience so much that no present-day AAA release can afford not to dazzle with alluring "RTX on" screenshots. What many people don't know: Raytracing is actually **much simpler** than "traditional" rendering!

![Final render animation](img/final-optimized.gif)

This repository contains code, assets, and presentation slides from an introductory course on raytracing. From **absolute scratch**, we build up a rendering engine that can handle a basic set of shapes and materials. Each part of the engine is easy to understand, and the final product offers a massive attack surface for straightforward extensions to continue working on.


Presentation slides
===================

* Based on revealJS
* Best viewed in Chromium browser at 1024x768
* The version in this repo requires internet connection to load JS dependencies

![first slide](slides/assets/img/first-slide-teaser.png)


Errata
------

This course is certainly not perfect. One known "issue" is that the Phong-like illumination model does not have a correct diffuse component, although this is not immediately noticeable.


Code
====

The code is split into "levels" which follow the structure of the course. Each level introduces a new concept and builds on the foundation of the prior levels.


Licenses
========

* The course slides are under [CC-BY-NC](https://creativecommons.org/licenses/by-nc/4.0/legalcode) license.
* Original non-content sources are MIT-licensed.
* The course builds upon third-party elements (frameworks such as reveal.js, libraries such as MathJax, fonts etc.). Each such element may have its own license.

