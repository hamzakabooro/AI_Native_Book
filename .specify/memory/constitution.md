<!-- Sync Impact Report:
  - Version change: N/A → 1.0.0
  - Modified principles: N/A (new constitution)
  - Added sections: Core Principles (6), Technical Standards, Development Workflow, Safety Requirements
  - Removed sections: N/A
  - Templates requiring updates:
    - plan-template.md ✅ updated
    - spec-template.md ✅ updated
    - tasks-template.md ✅ updated
  - Follow-up TODOs: None
-->
# Physical AI Engineering Book Constitution

## Core Principles

### Specification-First Development
All book content, code samples, and RAG chatbot functionality must begin with a clear, detailed specification before implementation. Every module must include objectives, architecture sketches, acceptance criteria, and testing strategies before writing content. This ensures comprehensive coverage of Physical AI, ROS 2, Gazebo, Unity, Isaac, VLA, HRI, and other key topics in a structured manner.

### Technical Accuracy in Robotics and AI
All technical content must be factually accurate and validated against real systems. Every ROS 2 example must compile with colcon, every Isaac and Gazebo instruction must match current stable versions, and all technical claims must reference verifiable sources (official ROS 2 docs, NVIDIA Isaac docs, academic papers, vendor datasheets). No hallucinations: every technical statement must match real APIs and systems.

### Reproducibility of all Builds and Deployments
Every code example, environment setup, and deployment instruction must be reproducible by readers. All examples must work in Ubuntu 22.04, ROS 2 Humble, Jetson Orin, or Isaac Sim environments. Each module must include validated architecture sketches and documented pipelines (ROS 2 → Gazebo → Isaac → Jetson → Robot) that work end-to-end with provided configurations.

### Clarity for Technical Audience
Content must be clearly written for students with an intermediate-to-advanced technical background. Writing level should be equivalent to senior undergraduate engineering textbooks. Every concept must be explained with sufficient context, diagrams, and step-by-step instructions to enable understanding and implementation of complex Physical AI and robotics systems.

### Safety and Correctness for Robotics
All content must prioritize safety and correctness, especially for hardware instructions and robot behaviors. Every hardware instruction must include disclaimers for safety, power, and physical risk. Unsafe robot behaviors must be clearly warned against, and any instructions that could cause harm must include appropriate safety protocols for locomotion, SLAM, motors, and physical interactions.

### Grounded RAG Chatbot Responses
The integrated RAG chatbot must provide answers strictly grounded in the book content. No ungrounded answers are acceptable - all responses must be traceable to specific passages in the user-selected book text. The system must pass grounding checks and maintain latency under 1s for local queries and under 3s for remote queries.

## Technical Standards
All robotics and AI claims must reference verifiable sources (official ROS 2 docs, NVIDIA Isaac docs, academic papers, vendor datasheets). Code examples must be runnable in Ubuntu 22.04, ROS 2 Humble, Jetson Orin, or Isaac Sim environment. Architecture diagrams must reflect real working pipelines (ROS 2 → Gazebo → Isaac → Jetson → Robot). All instructions for hardware must include disclaimers for safety, power, and physical risk. Book structure must match Spec-Kit Plus conventions: sections, units, acceptance criteria.

## Development Workflow
All content must follow the Spec-Kit Plus methodology with clear modules, objectives, architecture sketches, concepts summaries, code examples, testing strategies, and troubleshooting sections. Each module must document chunking strategy, embedding model selection, query pipeline, and evaluation methodology for the RAG system. Every implementation must include validated architecture sketches with PlantUML or Mermaid diagrams showing real working pipelines.

## Safety Requirements
No instructions that cause unsafe robot behavior without warnings are permitted. No unverified hardware instructions or incorrect voltages shall be included. Include safety notes for locomotion, SLAM, motors, and physical interactions. Clear disclaimer on real-world robot testing latency issues must be present. All physical AI implementations must include appropriate safety protocols.

## Governance
This constitution supersedes all other practices for the Physical AI Engineering Book project. All contributions must verify compliance with these principles. Any amendments to this constitution require documentation of the change, approval from project maintainers, and a migration plan for affected content. All PRs/reviews must verify compliance with technical accuracy, reproducibility, safety requirements, and grounding in the RAG system.

**Version**: 1.0.0 | **Ratified**: 2025-01-14 | **Last Amended**: 2025-01-14