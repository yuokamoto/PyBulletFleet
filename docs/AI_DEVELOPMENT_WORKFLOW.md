---
orphan: true
---

# AI-Native Development Workflow

**Status:** v0.1 — Initial working version

**Project:** PyBulletFleet

## 1. Purpose

This document defines the standard AI-native software development workflow for PyBulletFleet.

Its purpose is not to maximize the amount of code written by AI agents. It is to establish clear responsibility boundaries between humans and agents, preserve engineering judgment as implementation becomes cheaper, and create a development loop that produces reviewable evidence before changes are considered complete.

This document primarily defines:

- who owns which decisions;
- how a change moves from an idea to a merged implementation;
- when human approval is required;
- how scope and architectural decisions are controlled;
- how specifications, plans, evidence, reviews, and retrospectives relate;
- how the workflow itself evolves through repeated use.

This document is not a replacement for repository-specific instructions, coding conventions, CI, or reusable task procedures.

In general:

- `docs/AI_DEVELOPMENT_WORKFLOW.md` defines the development process and responsibility boundaries.
- `AGENTS.md` defines repository-wide instructions that agents should generally know.
- Agent-specific instruction files such as `CLAUDE.md`, when needed, contain only agent-specific guidance or compatibility instructions.
- Skills define reusable procedures for recurring classes of work.
- ADRs record durable architectural decisions and their rationale.
- Issues or equivalent task specifications describe individual changes.
- Pull requests describe the implemented change and provide evidence for review.

## 2. Core Principles

### 2.1 Human owns decisions; agents support them

Agents may investigate, reason, challenge assumptions, propose alternatives, draft specifications, and recommend decisions.

The human remains the owner of:

- the problem to solve;
- why it matters;
- priority;
- scope and non-goals;
- important architectural decisions;
- acceptance criteria approval;
- approval of material scope changes;
- the final definition of Done;
- the decision to merge.

Human ownership does not mean that the human must produce these artifacts alone. Collaborative reasoning with agents is encouraged. The distinction is that the final decision remains explicit and human-owned.

### 2.2 Plan before significant implementation

For non-trivial changes, implementation should not begin before the repository has been investigated and an implementation plan has been produced.

The plan should be proportional to the uncertainty and impact of the change.

### 2.3 Prefer a useful end-to-end capability per feature slice

A pull request should represent a change that can reasonably be understood,
reviewed, tested, and accepted as one concept. For feature work, prefer the
smallest **useful end-to-end user operation**, not the smallest code diff or
internal state profile. Tests may cover narrower profiles than the PR.

Lines changed are not the primary measure of size. Combining several states or
components is appropriate when they are needed for one reviewable user
operation; this is not permission for an unbounded feature PR.

Split work when there is a concrete reason, such as independent user value, a
material architecture boundary, compatibility risk, a blocking technical
unknown, or reviewability. State which end-to-end acceptance step each
intermediate PR advances and what remains before the feature goal is usable.

### 2.4 Evidence before Done

Agents should produce evidence that the agreed acceptance criteria have been satisfied.

Evidence may include:

- unit tests;
- integration tests;
- end-to-end tests;
- static analysis;
- type checking;
- benchmarks;
- scenario results;
- documentation checks;
- manual verification results where automation is impractical.

Passing tests do not themselves define Done. The human decides whether the evidence is sufficient.

### 2.5 Unexpected scope returns to the human

Agents may discover adjacent improvements during implementation.

If an additional change is not required to satisfy the approved goal or acceptance criteria, it should normally be recorded as a follow-up candidate rather than implemented opportunistically.

Agents may recommend scope expansion, but material expansion requires human approval.

### 2.6 Automate after repetition

Do not introduce agent infrastructure, specialized roles, skills, or automation solely because they might become useful.

When the same valuable instruction or workflow recurs approximately two or three times, consider promoting it into a reusable mechanism such as:

- `AGENTS.md`;
- an agent-specific instruction;
- a Skill;
- CI;
- a template;
- an ADR;
- an automated workflow.

The workflow is a tool for controlling engineering risk, not a process to satisfy for its own sake.

## 3. Responsibilities

Responsibility distinguishes decision ownership from execution.

| Activity | Human responsibility | Agent responsibility |
| --- | --- | --- |
| Problem definition | Own and approve | Investigate, clarify, challenge, draft |
| Priority | Decide | Provide evidence and alternatives |
| Goal | Own and approve | Clarify and draft |
| Scope / Non-goals | Own and approve | Draft, identify ambiguity and risks |
| Acceptance Criteria | Approve | Draft and refine |
| Repository investigation | Review when useful | Perform |
| External technical research | Evaluate relevance | Perform when useful |
| Architecture options | Decide important trade-offs | Investigate, propose, compare |
| Implementation plan | Approve when required | Produce |
| Detailed design | Review when useful | Produce |
| Implementation | Review | Perform |
| Tests / benchmarks | Judge adequacy | Implement and execute |
| Verification evidence | Evaluate | Produce |
| Independent review | Consider findings | Perform |
| Scope changes | Approve or reject | Identify and propose |
| Done / Merge | Decide | Report readiness and unresolved concerns |

Agents are expected to participate actively in reasoning. Human ownership should not become a requirement for the human to manually perform work that can be delegated safely.

## 4. Change Classification and Workflow Weight

Use the lightest workflow that adequately controls the risk of the change.

Apply this principle to the **whole feature theme**, not separately to every
controller, entity type or proof. The workflows below are examples, not a
ceremony to restart for each internal profile. Reuse approved scope and
architecture decisions while implementation stays within their boundaries.

Classification is based primarily on risk, uncertainty, architectural impact, compatibility risk, and review complexity rather than line count.

Implementation size and engineering risk are separate dimensions. A small diff may be high-risk (for example, a public API compatibility break), while a large test-only change may be relatively low-risk. Workflow weight should follow risk and uncertainty, not raw change size.

### 4.1 Small changes

Examples include:

- typo or documentation corrections;
- obvious localized bug fixes;
- isolated test improvements;
- trivial configuration changes.

Typical workflow:

`Problem → Implementation → Verification → Human Review → Merge`

A separate Issue, formal plan, ADR, or independent agent review is normally unnecessary.

### 4.2 Medium changes

Examples include:

- ordinary feature development;
- a new internal or public API with limited architectural impact;
- changes spanning several files or components;
- behavior changes requiring explicit acceptance criteria.

Typical workflow:

`Problem / Specification → Human Scope Approval → Plan → Implementation → Verification → Review → Human Final Approval → Merge`

### 4.3 Architectural or high-risk changes

Examples include changes affecting:

- core abstractions;
- public API compatibility;
- simulation semantics;
- the boundary between PyBulletFleet and related systems;
- ROS transport versus core APIs;
- major performance trade-offs;
- cross-component responsibility;
- long-lived architectural constraints.

Typical workflow:

`Specification → Human Scope Approval → Plan → Human Architecture Approval → ADR when warranted → Implementation → Verification → Independent Review → Human Final Approval → Merge`

An ADR or independent review is not required merely because a change is large. Use them when they improve decision quality or preserve important knowledge.

## 5. Standard Development Workflow

### 5.1 Discovery

Candidate work may originate from:

- a human idea;
- a reported problem;
- an existing roadmap item;
- repository investigation;
- external technology scouting;
- new research, standards, tools, or OSS developments.

Discovery does not imply implementation.

A candidate should first be evaluated for relevance to the core value and direction of PyBulletFleet.
Before placing an investigation, proof or evaluation on a feature's critical
path, ask whether its uncertainty actually blocks the end-to-end acceptance
demonstration or V1 delivery. Related work that does not block it belongs in
follow-ups, even if it would reduce uncertainty.

### 5.2 Problem Definition and Task Specification

Before significant implementation, establish enough information to make the intended change reviewable.

For a feature theme spanning multiple changes, define its Product Goal,
concrete user operation, end-to-end acceptance demonstration, supported V1
boundary and explicit later/non-goal items **before** deriving PR slices.
Each PR states how it advances that operation; an internal proof is not
automatically a product milestone. This information may live in an existing
Issue or design document rather than a new artifact.

A useful specification normally includes:

- **Context** — relevant background;
- **Problem** — what is currently inadequate;
- **Goal** — what outcome is desired;
- **Non-goals** — what is intentionally outside scope;
- **Constraints** — compatibility, architecture, performance, dependency, or other limits;
- **Acceptance Criteria** — observable conditions for success;
- **Evidence** — how the criteria are expected to be verified.

The human may start with a rough request. An agent may investigate the repository, ask questions, and draft the specification. The human approves the resulting scope.

### 5.3 Context Gathering

Before planning a medium or higher-risk change, the agent should gather the context needed to reason about the change.

Relevant context may include repository instructions, relevant source and abstractions, tests, related Issues/PRs, ADRs, workflow learnings when applicable, CI/benchmark infrastructure, and external sources when useful.

Context gathering should be targeted rather than exhaustive: reduce important unknowns before planning rather than mechanically reading the entire repository.

### 5.4 Planning

For medium or higher-risk work, planning follows context gathering and precedes implementation.

A plan should cover, as appropriate:

- relevant existing code and architecture;
- proposed change locations;
- interactions with existing abstractions;
- architecture implications;
- compatibility considerations;
- test and verification strategy;
- benchmark needs;
- risks and unknowns;
- possible decomposition into independently reviewable changes.

Plans should not become design documents by default. Include only the detail needed to make implementation direction and risk understandable.
If decomposing a feature, explain why each split is needed and which end-to-end
acceptance step it advances. Different controller or state profiles alone do
not require separate PRs or repeated approval gates.

### 5.5 External Research

Feature planning should include external research when it can materially improve the decision.

Potential sources include:

- relevant ROS projects;
- established robotics OSS;
- simulators;
- standards;
- recent papers;
- technical documentation;
- engineering discussions and credible industry material.

Research should answer a concrete question. It should not be performed mechanically for every change.

The agent should distinguish:

- established facts;
- approaches used by other projects;
- inferred lessons;
- recommendations for PyBulletFleet.

External precedent informs architecture decisions but does not replace project-specific reasoning.

### 5.6 Implementation

Once required approval gates have been passed, the agent may implement the approved plan.

During implementation:

- preserve the approved conceptual scope;
- follow repository instructions and existing conventions;
- add or update tests with the implementation;
- document unexpected constraints;
- avoid unrelated cleanup unless required for the approved change.

If implementation reveals a material assumption failure or scope expansion, return to the human rather than silently redefining the task.

### 5.7 Verification

The agent and CI should execute the relevant automated verification.

This may include:

- unit tests;
- integration tests;
- end-to-end tests;
- linting;
- type checking;
- benchmarks;
- simulation scenarios;
- compatibility checks.

The human does not normally need to execute automated tests manually.

Choose local checks by change risk and affected surface as described in
`AGENTS.md`. Focused tests support iteration; relevant checks support a push;
full verification applies to high-risk changes and source PRs presented for
final review. CI still runs repository-wide checks. Repeating the full suite
after an unchanged documentation edit is not evidence of greater safety.

The human reviews whether the tests and other evidence actually demonstrate the intended acceptance criteria.

Manual verification remains appropriate when important behavior cannot be validated adequately through automation.

After a pull request is created, the implementation agent should normally remain responsible for bringing the approved change to a reviewable green state: observe CI, investigate failures, fix failures caused by the change, rerun checks, and update evidence. Failures that imply material scope or architecture changes return to the human.

For performance-sensitive changes, a **benchmark gate** may compare the proposed change with an agreed baseline and threshold before merge. This is change-scoped verification and is distinct from continuous post-merge performance feedback.

### 5.8 Review

The implementing agent should review its own work before presenting it as ready.

For changes where independent review is useful, use a separate context or reviewer to evaluate the implementation without relying on the implementation conversation.

Independent review should use the review lenses relevant to the change rather than mechanically invoking every possible reviewer.

Useful review lenses include correctness, architecture, API compatibility, performance, ROS integration, simulation semantics, unnecessary complexity, test adequacy, scope compliance, and unresolved assumptions.

A change may need only a general review, while a performance-sensitive or ROS-facing change may benefit from an additional focused lens. Specialized reviewer agents or Skills should be introduced only after repeated use demonstrates value.

Independent agent agreement is evidence, not human approval.

### 5.9 Merge

Before merge, the agent should summarize:

- acceptance criteria and their status;
- verification evidence;
- important review findings;
- architecture implications;
- unexpected changes;
- unresolved issues or risks;
- follow-up candidates.

The human makes the final Done and Merge decision.

### 5.10 Retrospective

After meaningful changes, briefly evaluate the development process.

Useful questions are:

1. What required human judgment?
2. What could the agent have handled better or more independently?
3. Where did the workflow create unnecessary overhead or allow scope creep?
4. Should any repeated knowledge or procedure be promoted into `AGENTS.md`, an agent-specific instruction, a Skill, ADR, template, CI rule, or this workflow?
5. Did the feature advance a usable end-to-end operation, and did review,
   approvals, documents or repeated verification cost more than the value they
   added? If so, which step should be reused, combined or removed next time?

A retrospective does not need to produce a workflow change. `No workflow changes` is a valid result.

## 6. Human Approval Gates

### 6.1 Gate 1 — Scope Approval

Required for medium and higher-risk changes.

The human should be able to answer:

- Is this problem worth solving now?
- Is the goal clear?
- Are the non-goals sufficiently clear?
- Is the proposed scope one conceptual change?
- Would satisfying the acceptance criteria solve the intended problem?

Approval means that implementation planning may proceed. It does not imply approval of every implementation detail.
For an approved feature theme, do not repeat Scope Approval solely because
implementation moves to another internal state or controller profile. Return
when the Product Goal, supported scope or acceptance demonstration changes
materially, or an assumption underlying approval fails.

### 6.2 Gate 2 — Architecture Approval

Conditional.

Use this gate when the change introduces a meaningful architectural decision or long-lived trade-off.

Questions include:

- Is the proposed architectural direction acceptable?
- Does it preserve the intended responsibility boundaries?
- Are important trade-offs understood?
- Are compatibility consequences understood?
- Should this decision be recorded as an ADR?

Agents should normally present alternatives and evidence rather than asking the human to invent all options from scratch.
Reuse an approved architecture boundary across profiles. Return for approval
only for a material responsibility change, public API direction change, major
compatibility/performance trade-off, or invalidated architectural assumption.

### 6.3 Gate 3 — Final / Merge Approval

The human reviews the implementation at the level appropriate for its risk.

The review should consider:

- whether acceptance criteria were satisfied;
- whether verification actually tests the intended behavior;
- relevant E2E or integration evidence;
- independent review findings when used;
- architectural consequences;
- unexpected scope changes;
- remaining risks.

Human review does not require manually reproducing every operation performed by an agent or CI.

## 7. Development Artifacts

The workflow defines information that should exist; it does not require a separate document for every stage.

Avoid creating a documentation bureaucracy.

### 7.1 Roadmap / Candidate Backlog

Early ideas and possible future capabilities may remain in a lightweight roadmap or backlog.

A useful categorization is:

- **Now**
- **Later**
- **Not now**
- optionally **Watch / Investigate**

Not every idea needs a GitHub Issue.

### 7.2 Issue / Task Specification

Once a candidate becomes sufficiently concrete to investigate or implement, a GitHub Issue may become the task-level source of truth.

A useful structure is:

```text
Context
Problem
Goal
Non-goals
Constraints
Acceptance Criteria
Plan
Evidence
Follow-ups
```

GitHub Issues are encouraged for meaningful work but are not mandatory for every change.

Small changes may be represented entirely by a pull request. Exploratory work may begin in a local Markdown specification and later be promoted to an Issue.

### 7.3 Plan

The plan may live inside the Issue or task specification. Do not create a separate planning document unless its complexity justifies one.

### 7.4 ADR

ADR means **Architecture Decision Record**.

Use an ADR for architectural decisions whose rationale is likely to matter beyond the current task.

A minimal ADR should capture:

- Context;
- Decision;
- Alternatives considered;
- Consequences.

ADRs record decisions, not every design discussion.

### 7.5 Pull Request

A pull request should primarily describe what was actually implemented and provide review evidence.

It should reference the Issue or specification when one exists rather than duplicating it unnecessarily.

Useful PR information includes:

- implemented change;
- intentional deviations from the plan;
- verification performed;
- benchmark results when relevant;
- known limitations;
- follow-up work.

### 7.6 Evidence

Evidence should be connected to acceptance criteria whenever practical.

The objective is not to maximize the number of tests. It is to make the claim that the change works inspectable.
Before adding a separate spec, plan, evidence or validation document, identify
the distinct review decision or durable knowledge it provides beyond the
existing Issue, PR description and tests. Temporary proof results may stay in
the PR and tests; do not create a repository document per internal profile.

## 8. Scope Control

During implementation, classify newly discovered work using a simple rule:

**Required to satisfy the approved acceptance criteria → current change.**

**Not required → follow-up candidate by default.**

Examples of likely follow-up work include:

- unrelated refactoring;
- opportunistic API cleanup;
- additional GUI functionality;
- unrelated performance optimization;
- removal of unrelated deprecated functionality;
- adjacent features discovered during implementation.

Exceptions are allowed when keeping the work separate would be materially more dangerous or wasteful. Material exceptions require human approval.

## 9. Technology Scouting and Idea Discovery

PyBulletFleet should not depend only on technologies that the human happens to encounter.

Technology scouting may periodically investigate developments relevant to areas such as:

- robotics simulation;
- fleet simulation and management;
- ROS and Open-RMF ecosystems;
- Physical AI;
- robot learning and synthetic data;
- simulation architecture;
- relevant standards;
- notable OSS projects;
- research papers;
- credible engineering discussions and industry developments.

Potential discovery channels may include GitHub, academic sources, project documentation, technical blogs, and social/professional sources such as X or LinkedIn when they provide useful leads.

The scouting workflow is:

`Discovery → Candidate → Human Relevance Decision → Investigation → Human Roadmap Decision → Roadmap / Issue`

Scouting should produce candidates and evidence, not automatically create implementation work.

A useful candidate classification is:

- Ignore;
- Watch;
- Investigate;
- Candidate for roadmap.

Initially, perform scouting manually or on demand. Consider periodic automation only after the desired scope and output format have proven useful through repeated use.

## 10. Knowledge Promotion and Workflow Evolution

Retrospectives may identify information that deserves a longer-lived home.

### 10.1 Workflow Learning Record

The rule "automate after repetition" requires a lightweight memory of previous development experience. Do not rely only on human memory or individual agent conversation history to determine whether a pattern has occurred repeatedly.

Maintain a lightweight `AI_WORKFLOW_LEARNINGS.md` (or equivalent) that records only reusable AI-development observations and promotion candidates.

Create the record when the first reusable observation is identified. An empty file is not needed before then.

This is not an activity log or a transcript of agent usage. Do not record every agent session, prompt, pull request, or retrospective.

Record an observation when a retrospective identifies something likely to matter again, for example:

- an instruction that had to be given explicitly;
- a recurring agent failure mode;
- a review check that repeatedly found useful issues;
- a repeated verification or benchmark procedure;
- a repository rule that agents repeatedly failed to infer;
- a workflow step that repeatedly created unnecessary overhead;
- a useful procedure that may eventually deserve a Skill or automation.

A learning entry should contain enough information to recognize recurrence, such as:

```text
Pattern: Establish a benchmark baseline before performance work
Occurrences: 2

Evidence:
- #52: baseline had to be requested explicitly
- #61: same instruction was required again

Candidate destination:
- performance Skill

Status:
- promotion candidate
```

The exact format should remain lightweight.

### 10.2 Retrospective Learning Loop

During a retrospective:

1. Identify reusable observations from the completed task.
2. If a learning record exists, compare the observations with its existing entries.
3. Add a new pattern only when the observation is plausibly reusable.
4. Increment or update an existing pattern when it occurs again.
5. When a pattern has repeated enough to justify standardization—typically two or three meaningful occurrences—ask whether it should be promoted.
6. The agent may recommend a destination and draft the change.
7. The human decides whether promotion is warranted.

A typical loop is:

`Development → Retrospective → Workflow Learning → Recurrence Check → Human Promotion Decision`

Do not automatically modify repository-wide instructions, Skills, CI, templates, or this workflow merely because an occurrence threshold was reached. Repetition is evidence for consideration, not an automatic trigger.

### 10.3 When to Read Workflow Learnings

`AI_WORKFLOW_LEARNINGS.md` does not need to be loaded into every implementation-agent context.

Its primary use is during retrospectives and workflow improvement work, where the current observation can be compared with previous experience.

A task may consult relevant learnings earlier when they directly apply, but mandatory full-file review before every implementation is discouraged if it adds noise without improving decisions.

### 10.4 Promotion Destinations

Use the following rough mapping:

- repository-wide rule or invariant → `AGENTS.md`;
- agent-specific behavior → agent-specific instructions such as `CLAUDE.md`, only when necessary;
- recurring task procedure → Skill;
- durable architecture decision → ADR;
- automatically enforceable invariant → CI or test;
- task-specific information → Issue / PR;
- development-process rule → this document.

After promotion, the learning record may retain a short reference to what was promoted and where, rather than duplicating the authoritative content.

Avoid duplicating the same authoritative rule across multiple artifacts when a reference is sufficient.

## 11. Future Evolution

The following are potential future capabilities, not current requirements or implementation tasks:

- specialized Agent roles;
- a formal Skill catalog;
- automated task classification;
- agent-to-agent handoffs or protocols;
- job queues and asynchronous execution;
- automated independent review;
- continuous technology scouting;
- automated experiment workflows;
- research and algorithm-development agents;
- multi-agent orchestration.

Introduce these only when repeated experience demonstrates a concrete benefit.

### 11.1 Continuous Performance Feedback

A future continuous performance feedback loop may complement the pre-merge benchmark gate.

- **Benchmark gate:** change-scoped and pre-merge. It asks whether a specific proposed change satisfies an agreed performance threshold relative to a baseline.
- **Continuous performance feedback:** system-scoped and ongoing/post-merge. It repeatedly observes benchmark or scenario results over time, detects regressions or opportunities even when no current feature is being reviewed, investigates likely causes, and proposes follow-up work.

A possible PyBulletFleet loop is:

`Scenario / Benchmark Suite → Periodic Execution → Trend or Regression Detection → Agent Investigation → Root-Cause / Improvement Proposal → Human Decision → Issue`

Unlike a production-service performance factory, PyBulletFleet can use reproducible simulation scenarios, scale tests, and benchmark histories as its primary signals.

Do not build this automation until the benchmark suite, useful metrics, and repeated investigation workflow are stable enough to justify it.

### 11.2 Research / Algorithm Development Loop

PyBulletFleet may eventually support an AI-assisted loop for developing and evaluating fleet or robotics algorithms, for example:

`Hypothesis → Algorithm → Scenario Generation → Simulation → Benchmark → Analysis → New Hypothesis`

This is conceptually different from the software-development workflow defined in this document.

If this becomes a real use case, specialized roles such as research, experiment, implementation, or evaluation agents may become useful. Their responsibilities should be derived from actual repeated workflows rather than designed prematurely.

## 12. Non-Goals of This Workflow

This workflow is not intended to create:

- an autonomous software factory;
- a 24-hour autonomous development system;
- a mandatory multi-agent architecture;
- an agent-specific development process;
- a rigid state machine for every code change;
- mandatory documentation for trivial work;
- automatic implementation of every promising idea;
- a replacement for human engineering judgment.

The goal is to improve the quality and scalability of human-directed engineering while taking advantage of increasingly capable coding agents.

## 13. Initial Adoption Strategy

Adopt this workflow incrementally.

1. Use this document as the initial process definition.
2. Have a local coding agent inspect the existing repository, including existing instructions, Skills, templates, CI, and documentation.
3. Ask the agent to propose the minimum changes required to support this workflow before implementing them.
4. Have a human review that proposal for duplication and overengineering.
5. Implement only the approved minimum.
6. Select one real PyBulletFleet feature and run the workflow end to end.
7. Retrospect on both the feature and the workflow.
8. Repeat for two or three meaningful changes before introducing substantial orchestration or automation.

The workflow itself is expected to evolve based on evidence from real development.
