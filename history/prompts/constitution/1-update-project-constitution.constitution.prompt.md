---
id: 1
title: Update-project-constitution
stage: constitution
date: 2025-12-15
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, spec-driven-development, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
tests:
 -
---

## Prompt

```text
# Constitution of the Unified Spec‑Driven Book Project

## 1. Purpose and Vision

This document defines the **constitution** of the project: the non‑negotiable principles, architectural rules, roles, workflows, and quality bars that govern how the unified book, AI systems, and infrastructure are designed, implemented, and evaluated.

The project's mission is to **bridge digital intelligence and physical intelligence** by producing:

* A **spec‑driven technical book** on *Physical AI & Humanoid Robotics*
* A **deeply integrated AI assistant** that understands, personalizes, and translates the book
* A **reusable intelligence system** built with Claude Code, Spec‑Kit Plus, agents, subagents, and skills

This constitution ensures consistency, reproducibility, and extensibility across all contributors and automated agents.

---

## 2. Governing Principles (Non‑Negotiable)

### 2.1 Spec‑First Development

* **No implementation without spec**: Every feature starts with a complete specification
* **Markdown / MDX** – content format
* **GitHub Pages** – deployment target

### 3.2 Spec & Intelligence Layer

* **Spec‑Kit Plus** – requirements, contracts, and specs
* **Claude Code** – implementation, refactoring, and agent orchestration
* **Claude Agent Skills** – reusable capabilities
* **Claude Subagents** – specialized autonomous roles

### 3.3 AI & Backend Systems

* **FastAPI** – backend API
* **OpenAI Agents / ChatKit SDKs** – conversational orchestration
* **Neon Serverless Postgres** – structured data & auth profiles
* **Qdrant Cloud (Free Tier)** – vector storage

### 3.4 Authentication & Personalization

* **Better‑Auth** – signup/signin
* Profile‑driven personalization engine

---

## 4. System Architecture (Authoritative)

### 4.1 High‑Level Layers

1. **Specification Layer**
   Specs define chapters, agents, APIs, personalization rules, and UI behaviors

2. **Content Layer**
   Docusaurus book generated and maintained by Claude Code via specs

3. **Intelligence Layer**
   Claude agents, subagents, and skills executing reusable cognition

4. **Retrieval Layer**
   RAG pipelines powered by Qdrant + structured metadata

5. **Interaction Layer**
   Embedded chatbot, chapter buttons, personalization UI

---

## 5. Book Constitution

### 5.1 Book Structure Rules

* Each **Module = One Docusaurus Section**
* Each **Week = One Chapter**
* Each chapter must include:

  * Learning objectives
  * Conceptual explanation
  * Practical exercises
  * Agent‑generated summaries

### 5.2 Immutable Course Scope

**Course Title:** Physical AI & Humanoid Robotics
**Theme:** Embodied Intelligence in the Physical World

Modules:

1. ROS 2 – Robotic Nervous System
2. Gazebo & Unity – Digital Twins
3. NVIDIA Isaac – AI Robot Brain
4. Vision‑Language‑Action (VLA)

---

## 6. RAG Chatbot Constitution

### 6.1 Core Capabilities (Mandatory)

* Answer questions about **entire book content**
* Answer questions based on **user‑selected text only**
* Cite retrieved sections internally (hidden from UI)

### 6.2 Behavioral Constraints

* No hallucinations beyond retrieved chunks
* If context is missing → respond with clarification request

### 6.3 RAG Data Model

* Chunked by:

  * Chapter
  * Section
  * Concept type (theory, code, hardware, warning)

---

## 7. Agent System Constitution

### 7.1 Core Claude Roles

**Master Spec Agent**
Owns specs, validates consistency, rejects violations

**Book Author Agent**
Generates and refines chapters from specs

**RAG Engineer Agent**
Manages embeddings, retrieval quality, and grounding

**Personalization Agent**
Adapts content to user background

**Translation Agent (Urdu)**
Faithful technical translation with preserved meaning

---

## 8. Subagents & Skills (Bonus Intelligence)

### 8.1 Subagents

* ROS Explainer Agent
* Hardware Advisor Agent
* Sim‑to‑Real Risk Analyzer
* Capstone Project Mentor

### 8.2 Agent Skills

* Spec → Markdown generation
* Chapter refactoring
* User background inference
* Safety & hardware constraint checking

All subagents must:

* Be stateless or explicitly scoped
* Declare inputs/outputs in specs

---

## 9. Authentication & User Modeling

### 9.1 Signup Questions (Mandatory)

* Programming experience
* Robotics experience
* Hardware access (GPU, Jetson, Robot)
* Preferred language

### 9.2 User Profile Usage

* Adjust depth of explanations
* Recommend hardware paths
* Control personalization & translation

---

## 10. Personalization Rules

* Personalization is **opt‑in** per chapter
* Activated via button at chapter start
* Content variants must remain technically correct
* No dilution of core concepts

---

## 11. Translation Constitution (Urdu)

* Triggered per chapter via UI button
* Technical terms preserved or parenthesized
* No cultural reinterpretation
* Meaning > literal translation

---

## 12. Hardware & Lab Governance

### 12.1 Accepted Models

* On‑Prem RTX Workstations
* Cloud GPU (AWS g5 / g6e)
* Jetson Orin Edge Kits

### 12.2 Latency Rule

* **Never control physical robots directly from cloud**
* Cloud = training, Local = inference

---

## 13. Evaluation & Scoring Framework

* Base Functionality: **100 points**
* Claude Subagents & Skills: **+50**
* Auth & Personalization: **+50**
* Chapter Personalization Button: **+50**
* Urdu Translation Button: **+50**

---

## 14. Change Management

* Constitution changes require:

  * Updated spec
  * Agent validation
  * Explicit version bump

---

## 15. Final Authority

If ambiguity arises:

1. Constitution
2. Specs (Spec‑Kit Plus)
3. Automated Agent Validation
4. Human Review

This constitution is binding for all humans and machines participating in the project.
```

## Response snapshot

Updated the constitution file at `.specify/memory/constitution.md` with the comprehensive project constitution for the Unified Spec-Driven Book Project. The constitution defines governing principles, system architecture, book structure rules, RAG chatbot requirements, agent system roles, and other critical project governance elements.

Also updated the following template files to align with the new constitution:
- `.specify/templates/plan-template.md` - Updated Constitution Check section with specific compliance gates
- `.specify/templates/spec-template.md` - Updated functional requirements to include constitution compliance items
- `.specify/templates/tasks-template.md` - Updated foundational tasks to reflect constitution requirements

## Outcome

- ✅ Impact: Created comprehensive project constitution and updated dependent templates
- 🧪 Tests: No tests required for this configuration update
- 📁 Files: Updated constitution file and 3 template files to ensure consistency
- 🔁 Next prompts: Ready for feature specification and planning
- 🧠 Reflection: Constitution provides clear governance framework for the project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A