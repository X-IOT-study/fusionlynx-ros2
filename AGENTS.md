# AGENTS.md

Last updated: 2026-09-08

This file is the entry point for agents working in this workspace. Read it
before changing code, configuration, documentation, experiment records, or
research artifacts.

## 1. Project scope

This repository is the thesis-oriented ROS 2 integration workspace for the
MotionCore mecanum robot. The production pipeline is:

```text
MotionCore firmware -> SDK -> ROS 2 driver -> robot description / TF
-> wheel odometry + FAST-LIO -> state_fusion_backend -> fused /odom
-> RTAB-Map -> evaluation and thesis artifacts
```

FAST-LIO is the primary LIO frontend. ORB-SLAM3 is comparison-only and must
not replace the production FAST-LIO path.

## 2. Source and documentation boundaries

```text
src/motioncore/       MotionCore ROS 2 integration and protocol-facing code
src/FAST_LIO/         FAST-LIO frontend and UniLidar configuration
src/fusionlynx/       robot, SLAM, fusion, mapping, and bringup packages
docs/                 architecture, contracts, development, and experiments
```

Authoritative documents:

- System architecture: `docs/architecture.md`
- TF and physical extrinsics: `docs/tf.md`
- Fusion and estimator contracts: `docs/fusion.md`
- FAST-LIO and RTAB-Map: `docs/slam.md`
- Deployment, build, and runtime checks: `docs/deployment.md`
- Development and research workflow: `docs/development.md`
- Git staging and commit policy: `docs/development/git_workflow.md`
- Complete pre-split rule text: `docs/AGENTS.full.md`

Read the relevant document before changing behavior. Treat the complete
pre-split file as an archive and compatibility reference; topic documents are
the maintained locations for future edits.

## 3. Hard system contracts

- `state_fusion_backend` owns the dynamic `odom -> base_link` transform.
- RTAB-Map owns the dynamic `map -> odom` transform.
- Robot description / static publishers own `base_link -> sensor` transforms.
- FAST-LIO must not publish a competing `odom -> base_link` in the integrated
  system.
- There must be one authoritative publisher for each dynamic TF edge.
- `/cmd_vel` is the unified motion command interface.
- `/odom` is authoritative fused local odometry, not RTAB-Map output.
- Do not silently rename or repurpose established topics, frames, messages, or
  lifecycle behavior.
- Do not add a second production localization path or handwritten mapper
  without an explicit architecture decision.
- URDF/Xacro is the source of truth for physical sensor placement.

## 4. Modification rules

1. Inspect the owning package and its documentation before editing behavior.
2. For architecture, data-contract, experiment, or hardware changes, read the
   relevant Research Vault material first.
3. Keep changes local to the owning package and preserve public contracts.
4. Validate frame IDs, timestamps, covariances, QoS, and pose conventions at
   sensor and fusion boundaries.
5. Preserve lifecycle safety, fixed-lag behavior, health transitions,
   initialization, timeout handling, and recovery behavior.
6. Add focused tests for changed behavior and declare new dependencies.
7. Search affected publishers, subscribers, TF broadcasters, launch files,
   YAML files, tests, and documentation before changing an interface.
8. Do not use broad refactors to hide a local defect.
9. Only commit changes after they have been verified and accepted. Once a change passes acceptance, stage the related files with `git add` and create a clear `git commit` so that each accepted modification remains traceable.
10. Never use `git add .` or `git add -A`; stage an explicit file or directory
    allowlist as described in `docs/development/git_workflow.md`.
11. Do not reset, discard, or overwrite existing user changes. Do not commit
    unrelated work.
12. Do not commit build output, logs, raw bags, databases, point clouds,
    `.mineru_output/`, or unreviewed generated artifacts.

## 5. Required reporting

If a change affects motion control, TF, topic names, frame semantics,
lifecycle behavior, sensor extrinsics, fusion architecture, SLAM
configuration, experiment configuration, evaluation methodology, or paper
assets, update the matching topic document and Research Vault record in the
same working session.

Keep stable rules here, package implementation knowledge in package
`README.md`/`design.md` files, architecture contracts in `docs/`, and
experiment decisions/results in the Research Vault.

## 6. Priority order

When requirements conflict, use this order:

1. Safety and lifecycle correctness.
2. TF and frame correctness.
3. Sensor/topic contract correctness.
4. Timestamp and synchronization correctness.
5. State-estimation and fusion correctness.
6. Mapping consistency.
7. Reproducibility and experiment traceability.
8. Diagnostics and robustness.
9. Code cleanliness and refactoring.

## 7. Working tree layout

The workspace is `/home/dream/ros2_ws` and uses ROS 2 Humble. The Research
Vault is `/home/dream/Documents/work_xxxx/SLAM_Thesis_Vault`. Firmware and the
host SDK remain authoritative in `/home/dream/tools/MotionCore-Firmware` and
`/home/dream/tools/motioncore-client`.

Package-specific rules under `src/` are discovered by path. Start with
`src/fusionlynx/AGENTS.md` for FusionLynx package changes, then consult the
package README and the topic document named above.

## 8. Agent delegation and collaboration

### 8.1 General rules

- The main Claude Code agent is responsible for task understanding, planning, delegation, integration, and final reporting.
- Delegate specialized work to the most relevant available subagents when the task benefits from independent analysis or parallel investigation.
- Do not invoke every available subagent mechanically. Select agents according to task scope and expected contribution.
- Read-only investigation and review agents must not modify project files.
- Implementation agents may modify only files within the approved task scope.
- Do not claim that an agent was invoked unless the delegation actually occurred and its result was received.

### 8.2 Recommended delegation

- `codebase-explorer`: inspect repository structure, trace code paths, identify dependencies, and locate relevant documentation.
- `debugger`: investigate concrete defects, runtime failures, and inconsistent behavior.
- `test-engineer`: design and execute focused tests for changed behavior.
- `code-reviewer`: independently review implementation correctness, maintainability, and regression risks.
- `security-auditor`: inspect relevant security-sensitive code, permissions, resource handling, and trust boundaries.
- `documentation-expert`: review documentation consistency when architecture or interfaces change.
- `embedded-systems`: assist with MCU firmware, peripheral drivers, memory constraints, and embedded runtime behavior.
- `c-pro`: assist with C language correctness, memory management, and low-level system programming.

### 8.3 Development workflow

For substantial implementation tasks:

1. Explore the relevant packages, documentation, and existing implementation before proposing changes.
2. Establish the task scope, affected interfaces, constraints, and acceptance criteria.
3. Delegate independent investigation when it can materially improve understanding.
4. Implement only within the approved scope.
5. Ask `test-engineer` to validate changed behavior with focused tests.
6. Ask `code-reviewer` to independently inspect the resulting changes.
7. Involve `security-auditor` when the task touches security-sensitive boundaries.
8. Resolve confirmed findings and report unresolved issues explicitly.
9. Follow the existing Git staging and commit policy. Never commit before user acceptance.

### 8.4 Delegation boundaries

- Subagents must follow this file and all applicable package-level `AGENTS.md` instructions.
- A subagent's findings are evidence to evaluate, not automatic authorization to change architecture or public contracts.
- Parallel agents must not independently modify the same files unless explicitly coordinated.
- Preserve the existing AOCI workflow and do not allow agent prompts or recommendations to override its machine-issued plans, guides, or safety boundaries.
- Codex and other external coding tools are separate execution systems; do not claim they were invoked unless an actual integration or tool call was used.

<!-- aoci:begin -->
## AOCI 仓库认知

AOCI 为本仓库维护一个稳定、可版本化、可增量更新的仓库级认知层，供模型跨任务复用对系统的理解。

`aoci.txt` 是面向模型的结构化认知索引。它以每个受管理文件、数据库表或其他受管理对象一条独立 Entry 的方式，用符号标签与 F/R/A/S 语义表达对象的核心职责、重要关系、对外契约，以及理解或修改系统时必须知道的非显然约束和设计决策。

Header、目录段和全部 Entry 共同组成完整仓库索引，可以覆盖前端、后端、配置、数据库结构及其他受管理内容。受管理内容发生变化时，通常只需维护受影响的认知条目，不需要重新生成整个索引。

AOCI 提供系统架构、对象职责、重要关系、对外契约和关键约束的高密度视图。

### 工作原理

AOCI 采用“模型生成、模型读取”的认知闭环。

Header、Entry 和 Curation 语义的创作只按当前机器签发的 Plan 与实时 Guide 执行；由 Host 模型基于当前绑定证据独立完成。

Entry 的语义必须来自模型对真实证据的理解。不得仅依据路径、文件名、扩展名、AST、符号列表、依赖扫描、正则、固定模板或规则引擎推导、预填、拼接或改写索引语义。

对 Fresh Bootstrap，只按当前机器签发的 Plan 和实时 Guide 执行。当它们要求创作时，Host 模型创作 Root、Meta、Tag 和 F/R/A/S，提供 authoring-run 声明，并把它绑定到 Plan、Evidence 与完整 Candidate。不得要求 AOCI 填写 `origin=host_model`、制造 Receipt 或把程序生成的 Framework 当作语义。本文件不自行重建 Onboarding 流程。内部批次不是用户决策；只有遇到既有批准边界或真实的安全、漂移、CAS、Recovery 条件才停止。

### 最小使用入口

- `aoci_rules`：取得当前AOCI版本的会话运行合同。
- `aoci_overview`：建立或恢复本仓库的完整认知。
- `aoci_maintain`：受管理对象达到最终稳定状态后检查认知是否需要维护。
- `aoci_update_entry`：提交与当前证据和源码摘要绑定的完整语义更新批次。
- `aoci_report`：仅当当前布局和工具状态支持时，在证据不足、无法可靠生成语义时登记待办，不猜写。

其他MCP工具、CLI命令、参数和专项流程，以当前工具说明、Guide和 `--help` 返回内容为准，不在本文件中重复完整手册。

本区块只规定仓库接入、认知使用和收尾原则。`aoci_rules` 承载当前会话合同，Guide实时输出承载当前Plan的执行顺序与停点，工具Schema、Spec和Validator承载机器结构与判据；Prompt、Description、README和静态文档不能覆盖这些机器事实。

### 建立、生成和恢复认知

1. 每个新的 Agent Run 开始时，应先判断：

   - 本仓库是否已经存在可用的完整AOCI索引；
   - 当前上下文中是否已有与本仓库根、当前索引版本和当前AOCI服务相匹配，并且模型仍可可靠使用的完整仓库认知。

2. 仓库已经存在可用的完整索引，但当前Run没有可靠完整认知时，先调用 `aoci_rules`，再调用 `aoci_overview`。

   完整认知仍可靠时直接复用。局部不确定本身不要求机械重读系统全貌。

   可靠与否由机器事实决定，不由主观感觉决定：同一Agent Run内已完整加载过本仓库索引并确认交付，且此后`index_sha256`、`entry_count`与`cognition_receipt.mcp_service_version`均未变化，工具也未报告Dirty、Stale、未决Recovery或第三方正式字节冲突，即为可复用。新Agent Run、上下文压缩恢复、索引或服务版本变化，或上述任一状态出现，必须重读。无法给出这些事实时按未加载处理；不得仅凭“感觉不确定”重读系统全貌，也不得在事实未变时用重读替代判断。

   本Run从已知Host上下文压缩恢复时（包括宿主注入的压缩摘要），必须把此前模型认知视为不可靠。压缩handoff不得保留或摘要正式Whole-Index，也不得保留或摘要任何Overview Header、Entry、Chunk、Challenge或Attestation正文；只能保留安全续接所需的receipt身份、未完成write或Recovery状态，以及立即重载指令。复制进handoff的Whole-Index语义或receipt不能证明恢复后模型的当前认知可靠。若当前上下文已无法可靠保留运行合同，先调用 `aoci_rules`。继续业务任务前，使用 `refresh_reasons=["context_compaction"]` 和新的 `refresh_event_id` 调用普通完整Whole-Index `aoci_overview`（不设置 `check_only` 或设为false）；不得使用 `check_only` 或认知probe。原样跟随每个 `next_cursor` 直到 `completed=true`，确认交付，并且只基于新交付正文提交一次Attestation。完成这次新的完整传输后，即使Attestation为partial或fail也消费该generation，并按既有合同继续source-bound任务，不再自动调用第二次Overview。

   AOCI可以针对 `context_compaction`、项目 `cognition_refresh_threshold` 下的机器 `semantic_threshold` 或主要 `phase_transition` 提供checkpoint与认知状态事实。只需要这些紧凑事实时使用 `check_only=true`；这些事实只向Agent提供建议，不替模型决定是否需要系统全貌。

   Agent显式调用普通 `aoci_overview`（未设置 `check_only` 或为false）时，只要能形成一致的CognitionSet，AOCI必须完整交付请求scope。不得因为已有receipt、阈值未达到或没有待处理刷新原因而抑制正文。正式认知Dirty或Stale时仍交付正文，但必须标记不可靠。存在未决恢复或无法形成一致snapshot时失败关闭，不返回混合正文。

   普通Overview返回 `continuation_required=true` 时，必须原样提交 `next_cursor` 并自动继续到 `completed=true`。不得询问用户、开始业务任务或给出阶段性系统结论。Host截断、缺块、重复、乱序、cursor失败、Index变化或`chunk_tokens`变化时停止本次认知链。Attestation完成前不得用Memory、源码、Spec、`aoci.txt`、历史会话、scope、search或Entry读取修补或补充Whole-Index认知。Challenge ordinal是正式Entry序列中的1-based位置；Header内容、注释、空行、Section/Overview/Chunk Marker、Receipt与Metadata均不计数，Chunk Receipt ordinal使用同一序列。Attestation必须原样回绑本次Challenge发布的当前`index_sha256`、`entry_sequence_sha256`与`entry_count`；旧Index、旧Entry序列、旧数量或旧Attestation均无效。完整链结束后只正式提交一次既有模型认知Attestation；同一响应只允许一次不改变语义答案的JSON Schema或字段格式修正。对象、Tag或F不匹配即失败且认知吸收不确定，不得语义重试或旁路补答。首次认知失败时还不得执行Root/Meta、Migration、全局布局或其他未重新绑定的系统级决策。上下文压缩刷新若传输完整、认知身份不变、治理对齐且没有Recovery或第三方冲突，即使Attestation为partial或fail也消耗该refresh generation，并继续原任务，不再自动重读Overview。`system_mastery_percent`只自评系统框架——架构、职责、强关系、稳定外部契约以及高熵安全和维护约束——不表示完整实现或运行实况知识；机器索引覆盖率必须分开。默认只向用户输出由本次真实覆盖率、Challenge、块数、Token和掌握度生成的规定成功或失败一句话。Host截断时提示用户把 `overview_delivery.chunk_tokens` 设置为更小的合法值后重新开始，不得自动修改。

   加法认知等级必须与严格证明字段分开解释。`delivery_verified`表示已加载Index且Host交付已确认，但完整认知验证仍未完成；应表达为“已加载且交付已验证”，不得描述为“没有认知”或“没有理解系统”。`cognition_verified`要求Attestation通过（Challenge至少80%的ordinal完全正确且对象身份至多失手一处），`cognition_governed`还要求治理对齐。通用完整读取失败句只用于真实交付故障。

   当Overview响应包含可选`cognition-state/v2`投影时，必须分别解释各维度。其Level止于`model_cognition_usable`；`strict_attestation_verified`、`governance_aligned`与`current_system_cognition_reliable`都是独立状态，绝不参与该Level。ordinal、对象身份、Tag或核心F不匹配可以导致严格Attestation失败，而模型认知仍然可用；不得仅凭这种不匹配就宣称模型没有理解系统。只有`current_system_cognition_reliable=true`允许无保留地声称当前完整系统认知可靠。投影缺失时继续使用上述Legacy解释。

   普通的只读审计、分析、检查、不修改代码或不提交、不push，不自动等于严格零写入，也不改变上述认知有效性判断。Codex Memory和历史Skill只能辅助恢复经验、用户偏好与调查方向，不能替代与当前仓库根、索引摘要、AOCI服务身份和认知范围匹配的当前认知收据；项目AGENTS和当前AOCI身份在AOCI状态上优先于历史Memory。

   只有用户明确禁止Ledger、元数据、`.aoci`运行资产及任何文件写入时，才按严格零写入处理。若必要的认知建立与该边界冲突，必须报告冲突并请求用户裁决或建议使用隔离副本，不得静默以Memory替代当前仓库认知。

3. 仓库没有可用的完整索引，或当前只有最小骨架、Header不完整、Entries未完成、必要Curation尚未裁决时，如果需要建立正式完整AOCI索引，先取得 `aoci_rules`，然后进入当前AOCI Guide。由Guide依据仓库真实状态决定下一阶段并完成必要安全步骤。

   `aoci_maintain` 不替代索引建立流程。

   不在本文件中自行重建或硬编码完整索引生成状态机。

4. 在长程任务中，模型负责保留当前认知收据并正确使用刷新门禁：

   - Host报告上下文压缩或模型已知系统全貌丢失时，执行上述强制 `context_compaction` 重载规则；AOCI不能自行推断Host事件；
   - 进入真正的主要阶段时声明 `phase_transition`，不得把函数、测试运行或小步骤当作阶段；
   - 在有用的稳定检查点通过 `check_only=true` 取得机器语义计数；
   - 除已知压缩的强制重载外，由Agent判断当前任务是否需要再次显式获取指定scope或完整Overview；
   - 在维护和对齐完成前，保留AOCI报告的Dirty或Stale可靠性状态。

### 任务收尾与认知维护

5. 纯只读问答、分析、版本核验，或没有产生受AOCI管理对象变化的任务，不需要调用维护工具。当前AOCI版本是任意`aoci_overview` check_only或`aoci_maintain`响应里的`cognition_receipt.mcp_service_version`；二进制路径是项目`.mcp.json`里的`command`，CLI不必在PATH上。

6. 发生受AOCI管理对象变化时，待其达到本次任务的最终稳定状态后，只调用一次 `aoci_maintain`。不要在每次中间修改后逐文件维护。

7. 若维护结果返回真实语义候选，Host 模型必须基于每个候选绑定的对象和必要证据，独立创作完整标签与F/R/A/S更新。通过 `aoci_update_entry` 一次提交当前机器签发批次的完整候选集合，同时原样保留每项 `source_sha256`、`candidate_id` 与对应domain批次身份。`max_entries`只限制单次请求和原子事务，不限制logical plan、Whole-Index或Managed Scope。`remaining`非零时，在当前批次成功Apply后重新调用Maintain并从新preimage继续；绝不能为满足transport上限缩减Index覆盖或自行截取返回批次。

   没有足够证据且当前布局支持 `aoci_report` 时，使用它而不猜测、套用模板或为消除待办而生成缺乏证据的认知。

8. 必须遵守工具返回的结构化状态和安全边界：

   - `repair_required`：只修复明确命中的候选，再重新提交当前机器签发的完整批次；
   - `stopped`：结束当前写入尝试并检查 `failed_step`、错误、正式写入证据与Recovery。auto模式下，已证明零写入则记录closure并重新Plan；完整Intent和可证明postimage则Resume；策略要求Rollback且preimage可证明则精确恢复后重新Plan。只有证据不足、第三方正式字节冲突、需要审批或外部动作，或命中其他真实安全边界时，才停止整个用户任务；
   - 冲突、审批、人工裁决、权限和安全信号不得忽略；
   - 已经对齐后不得重复维护或重复写入；`refresh_ready_for_overview` 是checkpoint事实，由Agent决定是否为下一阶段请求普通完整Overview。

   维护完成后如果又修改了任何受管理对象，之前的维护结果失效，应在新的最终稳定状态重新完成收尾。

9. 用户只限制业务文件范围，但没有明确禁止仓库托管资产时，AOCI托管资产可以在收尾阶段为保持认知一致而更新，并应在审计和提交中与业务文件区分。

   用户明确禁止修改 `aoci.txt`、`.aoci`、元数据或任何额外文件时，以用户限制为准，不得写入，并如实报告剩余不一致。

### 专项流程

初始化、完整索引生成、Header生成、Entries生成、数据库结构索引、Curation、人工评审和故障恢复，只按当前AOCI Guide或工具在对应阶段返回的指令、命令和安全停点执行。

不预加载、不猜测，也不自行重建这些专项流程。平台调用方式、请求格式、批次上限、审批规则、索引格式细节和恢复步骤由对应Guide、工具说明、模型Prompt和CLI帮助按需提供。
<!-- aoci:end -->


=== 范围约束(约束你提议什么修法,不约束你找什么)===
凡是这里真的有问题,都要报——包括听起来罕见但本项目确实会产生的情况。
然后把修法收在范围内:
1. 这不是一篇安全攻防论文。可以校验,禁止过度防御。除非本项目另有说明,默认操作者是
   自己机器上的合作者;如果它真有对手,它会写明,以那个范围为准。
2. 不要加哈希/校验和/指纹,除非它替代了一个实质上更贵的操作,并且结果会改变下一步做什么。
3. 禁止防御性脚手架:不为这里不会发生的情况加 feature flag、迁移框架、兼容层、包装层。
4. 禁止钻牛角尖:冷门编码、符号链接竞态、RTL 文本、毫秒级竞态一律不在范围内,
   除非该情况经由本项目**受支持的用法**可达——它的文档示例、它公开的接口、它真实的
   数据。可达即可,不需要你复现出来;但"理论上构造得出"不算。
5. 该判断的地方就判断,不要换成评分表、检查清单,或对已经定论的东西再跑一遍校验。
6. 以上都不覆盖用户、本项目自己的约定、或更高优先级规则明确要求的安全、迁移、校验与
   审阅。那些是被要求的,是活儿本身,不算范围外。
7. 交付物的文字不是答辩记录。成立的就直说;注意事项集中到专门一节
   (Limitations、已知问题),别在每段末尾都缀一句免责;更不要把指令**或过程**
   写进成品:「别提 X」意味着 X 不存在(不是写「我们不讨论 X」);中间报错、
   被放弃的方案、改稿痕迹同样不是内容。围绕最强的结果组织——论文是发布会,
   不是工作汇报。不占优的数字,是取舍就说成取舍,不是就平实陈述,都不写成认输;
   数字本身照样留在表里。
8. 推进本身是交付的一部分。几个方案都合理时,选一个并写明取舍——只有分歧
   真的大、或选错难回头时才升级为提问。不挡住目标的细节,记下来接着走,
   不是先停下解决它。为了不选错而停在原地,本身就是选错。
9. 不留任何生成/纠偏痕迹:最终产出(代码、注释、docstring、commit message、
   PR 描述、总结)只呈现正确的最终结果,当作一开始就是这么写的——不提中间报错、
   试错纠偏、被放弃的方案、"为什么没做 X";也不出现 AI 辅助的标记,比如
   "Generated by / Co-authored-by: Claude/Codex" 署名、模型腔过渡语
   ("值得注意的是"、"综上所述"、"首先...其次...最后")、"以下是..."式开场白,
   措辞详略贴合项目和文档已有风格。
已经见过的形状,供你校准。是例子不是清单——一个真问题不会因为"长得像其中一条"就被驳回:
  H  为了比对两个表格的差异,给每一行都算哈希——直接比单元格就能回答
  H  写下一堆校验和文件,而没有任何代码会去读它们
  E  给一个没有用户、没有部署的应用做账号安全加固
  R  用一整夜对自己的补丁反复审计,而功能一行没写
  R  一个对任何提交都给不通过的审阅者
  O  一层守卫的理由是上一层守卫,而不是需求
另有两种长得像上面、但不是的。这些要报:
  ✓  用摘要比对来跳过重读一个你已经有的大文件
  ✓  本项目自己的文档示例就会产生的那种"听起来罕见"的输入
跑任何检查之前先回答:这次运行会检测出什么具体的失败?真出现了我下一步会做什么不同的事?
答不上来就别跑。
对的就说对。不要为了交差硬找问题。
