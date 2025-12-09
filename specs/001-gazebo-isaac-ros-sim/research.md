# Research: Gazebo-Isaac-ROS Simulation

## Decision: Simulation Engine Choice - Gazebo vs. Isaac Sim vs. Unity

### Rationale:
For the educational content, we need to select the most appropriate simulation environments based on realism, GPU requirements, and ease of use for students. Based on research:

1. **Gazebo** (now Ignition Gazebo/Harmonic):
   - Pros: Open-source, widely adopted in ROS ecosystem, excellent for physics simulation fundamentals, lower hardware requirements
   - Cons: Less photorealistic than Isaac Sim, basic rendering capabilities
   - Best for: Module 2 - Digital Twin fundamentals, sensor simulation, basic robot modeling

2. **NVIDIA Isaac Sim**:
   - Pros: Highly photorealistic, synthetic data generation, hardware-accelerated perception with Isaac ROS
   - Cons: Requires NVIDIA GPU, higher hardware requirements, commercial licensing aspects
   - Best for: Module 3 - Advanced perception, synthetic data generation, realistic sensor simulation

3. **Unity**:
   - Pros: High-fidelity graphics, cross-platform support, rich visualization capabilities
   - Cons: Not native to ROS ecosystem, requires additional integration layers, steeper learning curve
   - Good for: Visualization components, but not primary simulation environment

**Decision**: Use Gazebo for foundational concepts and Isaac Sim for advanced topics. Unity integration is limited to visualization components only.

### Alternatives considered:
- Custom physics engines: Would increase complexity and reduce compatibility with ROS ecosystem
- Web-based simulators (like CoppeliaSim with ROS integration): Less performance and fewer robotics-specific features

---

## Decision: RAG Pipeline Design - Embedding Model, Chunk Size, Retrieval Strategy

### Rationale:
For the RAG chatbot integrated with the educational content, we need to balance accuracy and latency:

1. **Embedding Model Considerations**:
   - Sentence-BERT models (all-MiniLM-L6-v2): Good balance of speed and accuracy, suitable for technical content
   - Instructor models: Better for domain-specific retrieval, more accurate for technical queries
   - OpenAI embeddings: High quality but requires API access, may be costly for students

2. **Chunk Size Strategy**:
   - Smaller chunks (200-400 tokens): Better precision, more relevant context retrieval
   - Larger chunks (500-800 tokens): More context but potential for irrelevant information

3. **Retrieval Strategy**:
   - Vector similarity: Good for semantic matching of technical concepts
   - Hybrid search: Combines keyword and semantic search for better results
   - Multi-step retrieval: Hierarchical approach (section → paragraph → sentence) for complex queries

**Decision**: Use sentence-transformer models for embedding (to maintain offline capabilities), implement hybrid search approach with 300-400 token chunks, and implement multi-step retrieval for complex technical queries.

### Alternatives considered:
- Dense passage retrieval with fine-tuning: Higher accuracy but requires more computational resources and training data
- Graph-based retrieval: Good for connected concepts but complex to implement for educational content

---

## Decision: Deployment Approach - On-Prem vs Cloud GPU Instance

### Rationale:
For students learning robotics simulation, deployment approach affects accessibility and learning outcomes:

1. **On-Prem Ubuntu Workstation**:
   - Pros: Matches target deployment environment (Ubuntu 22.04, ROS 2 Humble), no recurring costs, full control
   - Cons: Hardware requirements for Isaac Sim (NVIDIA GPU), setup complexity for beginners
   - Best for: Advanced students with proper hardware, ensuring reproducibility of results

2. **Cloud GPU Instance**:
   - Pros: Accessible to students without high-end hardware, preconfigured environment
   - Cons: Recurring costs, network dependency, less hands-on with environment setup
   - Good for: Students without suitable local hardware, temporary access for specific modules

**Decision**: Focus on on-prem Ubuntu workstation as primary approach (aligns with project constraints), provide cloud alternatives as secondary option with setup guides.

### Alternatives considered:
- Containerized environments (Docker/Podman): Good for reproducibility but may require more advanced Docker knowledge
- Virtual machines: Less efficient but more familiar to some students

---

## Decision: Hardware Abstraction - Jetson Examples vs Simulation Only

### Rationale:
Balancing between real hardware examples and simulation-only content affects accessibility:

1. **Jetson-Based Examples**:
   - Pros: Matches real-world deployment scenarios, hardware acceleration for perception
   - Cons: Hardware cost barrier for students, platform-specific code
   - Best for: Advanced modules focusing on deployment and optimization

2. **Simulation-Only Examples**:
   - Pros: Accessible to all students, easier debugging, consistent environment
   - Cons: Less realistic, may not expose hardware-specific challenges
   - Good for: Learning fundamentals and concepts

**Decision**: Primary content in simulation with references and code examples for Jetson deployment in advanced sections. This ensures accessibility while maintaining real-world relevance.

### Alternatives considered:
- ROS 2 agents for different platforms: Would increase complexity without significant learning benefit
- Hardware-in-the-loop (HIL) simulation: Too complex for target audience level

---

## Decision: Documentation Depth - Conceptual vs Step-by-Step Tutorial

### Rationale:
Balancing between high-level concepts and detailed tutorials based on target audience:

1. **High-Level Conceptual**:
   - Pros: Better understanding of principles, reusable knowledge, less maintenance
   - Cons: May not provide sufficient guidance for implementation
   - Good for: Theory, architecture, design patterns

2. **Step-by-Step Tutorial**:
   - Pros: Clear, actionable guidance, easier for students to follow
   - Cons: Higher maintenance with software updates, may discourage independent thinking
   - Good for: Getting started, practical implementation

**Decision**: Hybrid approach with conceptual explanations followed by step-by-step tutorials. Each module will include both architectural understanding and practical implementation.

### Alternatives considered:
- Reference documentation only: Not suitable for learning-focused content
- Video tutorials: Complementary but not replacement for written documentation

---

## Decision: Code Format - ROS 2 Packages vs Inline Examples

### Rationale:
Organizing code examples affects maintainability and learning:

1. **Standalone ROS 2 Packages**:
   - Pros: Complete, testable, follows ROS 2 conventions, reusable
   - Cons: More complex to manage, requires understanding of ROS 2 workspace structure
   - Best for: Complete examples, end-to-end implementations

2. **Inline Examples**:
   - Pros: Contextual, easier to understand, less overhead
   - Cons: Not directly executable, harder to test
   - Good for: Focused code snippets, specific concepts

**Decision**: Primary approach using standalone ROS 2 packages with inline code snippets for focused explanations. Packages are organized per module and use case.

### Alternatives considered:
- Jupyter notebooks: Good for interactive learning but not standard for ROS 2 development
- Colab notebooks: Cloud-based but less relevant to target ROS 2 development environment