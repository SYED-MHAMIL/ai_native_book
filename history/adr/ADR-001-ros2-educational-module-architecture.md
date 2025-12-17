# ADR-001: ROS 2 Educational Module Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-16
- **Feature:** 01-ros2-book-module
- **Context:** Need to create educational content for ROS 2 as the middleware nervous system of humanoid robots, targeting advanced Computer Science students, Robotics Engineers, and AI Engineers with Python experience. The module must serve as foundational knowledge for subsequent modules on simulation and AI integration.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Create a Docusaurus-based educational module with Python-first ROS 2 approach, focusing on humanoid robotics concepts and following a docs-first learning progression:

- **Documentation Platform**: Docusaurus (static site generator with search, navigation)
- **Content Format**: Markdown with Docusaurus front-matter for proper structure
- **Target Language**: Python with rclpy (ROS 2 Python client library)
- **Architecture Focus**: Three-layer architecture (AI Layer → Middleware Layer → Control Layer)
- **Educational Approach**: Docs-first → Code-second flow with progressive complexity
- **Robot Focus**: Humanoid-specific examples and use cases
- **Diagram Format**: Text-based diagrams for version control and portability

## Consequences

### Positive

- Docusaurus provides excellent navigation, search, and maintainability for educational content
- Python-first approach leverages target audience's existing Python knowledge
- Three-layer architecture clearly separates concerns for better understanding
- Humanoid-specific examples provide concrete, relevant learning scenarios
- Text-based diagrams ensure version control compatibility and proper rendering
- Progressive learning flow reduces cognitive load and improves comprehension
- Module continuity ensures smooth transition to simulation and AI modules

### Negative

- Docusaurus introduces additional build and deployment complexity compared to simple Markdown
- Python may have performance limitations compared to C++ for real-time applications
- Humanoid-specific focus may limit applicability to general robotics concepts
- Text-based diagrams are less visually appealing than image-based diagrams
- Docs-first approach may delay hands-on coding experience
- Dependency on ROS 2 Humble Hawksbill may limit compatibility with other versions

## Alternatives Considered

Alternative A: Generic Markdown documentation with static HTML hosting
- Framework: Jekyll or GitHub Pages
- Approach: Simple Markdown files with basic styling
- Why rejected: Lacks navigation features, search capability, and professional documentation experience needed for complex technical content

Alternative B: C++-first ROS 2 approach with rclcpp
- Framework: C++ with rclcpp client library
- Approach: Performance-focused with direct ROS 2 API access
- Why rejected: Target audience has Python experience, making C++ approach create higher learning barrier

Alternative C: General robotics focus instead of humanoid-specific
- Approach: Generic robotics examples applicable to all robot types
- Why rejected: Humanoid robotics provides more complex and relevant examples for AI integration and physical systems

Alternative D: Image-based diagrams instead of text-based
- Approach: Professional diagrams created in graphics software
- Why rejected: Images are harder to version control, modify, and maintain consistency across documentation

Alternative E: Code-first → Docs-second approach
- Approach: Hands-on coding immediately followed by explanations
- Why rejected: Complex ROS 2 concepts require conceptual understanding before implementation to avoid confusion

## References

- Feature Spec: ./specs/01-ros2-book-module/spec.md
- Implementation Plan: ./specs/01-ros2-book-module/plan.md
- Related ADRs: None
- Evaluator Evidence: ./history/prompts/01-ros2-book-module/