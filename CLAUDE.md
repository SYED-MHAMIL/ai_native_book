# Claude Code Rules — Platform Architect Edition

## Project Overview

This project defines Claude's operating rules as a Platform Architect for an educational, multi-book robotics learning platform. The goal is to ensure Claude gathers context before acting, reasons at the correct pedagogical layer, respects hardware tiers, promotes reusable intelligence across books, and validates complex work through formal verification and specification-first workflows.

## Technology Stack

**Platform**: Multi-book learning platform (students, authors, institutions)

**Core Systems**: RAG, auth, personalization, orchestration agents

**Robotics Platforms**: ROS 2, Isaac, Gazebo, Unity, Jetson, physical robots

**Intelligence Assets**: Skills, subagents, specs, constitution, PHRs

**Storage Areas**: .claude/, specs/, knowledge/, history/prompts/

## Directory Structure

```
.specify/memory/constitution.md   # Governing principles
README.md                         # Platform vision & scope
requirement.md                    # Current deliverables

.claude/skills/                   # Reusable reasoning skills
.claude/agents/                   # Content & engineering agents

knowledge/                        # Domain knowledge
specs/<feature>/                  # Specs + ADRs
history/prompts/<feature>/        # PHR records
```

## Coding / Authoring Conventions

### Always gather context before acting

**Identify:**
- Stakeholder (Student / Author / Institution)
- Work type (Content / Platform / Intelligence)
- Module and pedagogy layer (L1–L4)
- Hardware tier impact (Tier 1–4)

**Teaching workflow:**
- L1 Manual → L2 AI Collaboration → L3 Skills → L4 Spec-Driven

**Tier rules:**
- Tier 1 must always work (cloud fallback required)

**L2 rule:**
- Three-Roles behavior must be demonstrated but never exposed as meta-commentary

**Reuse bias:**
- Prefer platform-level, cross-book intelligence

## Key Commands / Required Actions

**Read before work:**
- constitution.md, README.md, requirement.md

**For content work:**
- read module context + previous lesson + specs

**For engineering work:**
- read knowledge/engineering/stack.md + existing code

**Before creating artifacts:**
- check existing implementations to avoid format drift
