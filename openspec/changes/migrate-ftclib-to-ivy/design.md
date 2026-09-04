## Context

Ver `proposal.md — Why` para a motivação e `MIGRACAO-IVY.md` na raiz do repositório para o
inventário completo e a referência da API do Ivy.

Estado atual relevante: 6 subsistemas `extends SubsystemBase`, 19 comandos `extends CommandBase`
(ou grupos), 2 OpModes `extends CommandOpMode`, tudo orquestrado por `RobotContainer` (fiação +
default commands + bindings `GamepadEx`/`GamepadButton` + getters de autônomo). Pedro Pathing
(`com.pedropathing:ftc:2.1.2`) já é dependência e já provê `Follower`, `Pose`, `PathChain`,
`com.pedropathing.control.PIDFController`. O `master` não compila.

Restrições do Ivy que moldam a abordagem (lidas do fonte, `github.com/Pedro-Pathing/Ivy`):
- Não há classe `Subsystem`, "default command", nem DSL de bindings.
- `Scheduler` é estático/global; exige `reset()` explícito por OpMode.
- `requirements()` aceita qualquer `Object`; conflito resolvido por `priority()` +
  `InterruptedBehavior` (`END`/`SUSPEND`); comandos suspensos são retomados pelo próprio
  `execute()` sem re-chamar `start()`.
- Comando sem requirements nunca conflita.

## Goals / Non-Goals

**Goals:**
- Framework command-based único (Ivy); zero `com.arcrobotics.ftclib` no código e no build.
- Paridade de comportamento observável: bindings, rotinas de autônomo e laços contínuos.
- Base compilável ao fim da migração; janela de não-compilação contida entre as fases 2–6.
- Padrões de conversão repetíveis (A/B/C/D) para reduzir decisão caso-a-caso.

**Non-Goals:**
- Refatorar a lógica interna de controle (PIDF, polinômios de distância, máquinas de estado de
  tiro) além do necessário para trocar de framework.
- Corrigir bugs pré-existentes (ternários de aliança, `IntakeConstants` morto, import perdido
  em `RedRearPoses`) — apenas anotar.
- Migrar OpModes de tuning (`Tuning.java`, `MotorDirections.java`) — não usam FTCLib.
- Redesenhar o autônomo ou trocar o esquema de poses indexadas por ordinal.

## Decisions

### D1 — "Default command" via comando `infinite` + prioridade + `SUSPEND`

O Ivy não tem default command. Alternativas consideradas:
- **(a) Chamada de método direta no `loop()`** (padrão do repo de exemplo `CompetitionTeleOp`):
  simples, mas perde a semântica de interrupção/retomada que o alinhamento ao AprilTag depende.
- **(b) Re-agendar manualmente o contínuo quando nada o reserva**: exige rastrear estado no
  OpMode, frágil.
- **(c escolhida) Contínuo como `infinite` com `requiring(subsistema)`, prioridade 0,
  `InterruptedBehavior.SUSPEND`; comandos de botão com prioridade ≥ 1.** O `Scheduler` retoma
  o suspenso automaticamente ao liberar o recurso — reproduz exatamente o default command da
  FTCLib com mecanismo nativo do Ivy.

O `LedCommand` é caso à parte: nada conflita com o LED, então é `infinite` **sem** `requiring`,
agendado junto dos `periodic()`.

### D2 — `periodic()` vira `Command periodic()` sem requirements

O corpo do `periodic()` atual (leitura de sensores, `follower.update()`, telemetria, aplicação
de estado) roda **sempre**, inclusive enquanto um comando de ação detém o subsistema. Logo o
comando `periodic()` **não** pode reservar o subsistema. Fica `Commands.infinite(...)` sem
`requiring`, agendado uma vez no `init()`. Só o `Scheduler.reset()` o encerra.

### D3 — Comandos como fábricas `static`, não classes

Padrão do Ivy (`Command.build()` + `Commands`/`Groups`). Classes `extends CommandBase` viram
classes utilitárias com métodos `static` que retornam `com.pedropathing.ivy.Command`.
Vantagem: comandos stateless, compõem sem estado residual. Para os 3 comandos com estado
mutável entre loops (`ShootCommand`, `ActiveAimCommand`, `GoToPoseCommand`): capturar o estado
em objeto/array `final` local no builder, ou implementar a interface `Command` diretamente
(Class API) — decidir por comando na Fase 3.

### D4 — Bindings por polling no `loop()`, sem `GamepadEx`

`GamepadButton`/`Trigger` são FTCLib. O SDK 11.2.1 já tem edge-detection nativa
(`gamepad1.yWasPressed()`/`yWasReleased()`). Bindings saem do `RobotContainer` e vão para o
`loop()` do OpMode como `if`s. `whileHeld(cmd)` → `if (pressed) cmd.schedule(); if (released)
cmd.cancel();`.

### D5 — `RobotContainer` → `Robot` (fiação) + OpMode (bindings)

`RobotContainer` mistura fiação, poses, laço de resync, bindings e getters de autônomo.
Separação: `Robot.java` só constrói subsistemas e expõe `periodicCommands()`, `shooterAutoAdjust`,
`isShooting()`, helpers de pose. Bindings e laço de resync vão para `teleop.java`. Getters de
autônomo viram fábricas `static` (`AutoRoutines`).

### D6 — OpMode base iterativa (`extends OpMode`)

`CommandOpMode` é FTCLib e é `LinearOpMode` (bloqueante). Nova base `RobotOpMode extends OpMode`
com `init`/`init_loop`/`loop`/`stop`. Consequência: a configuração pré-play de `Autos.java`
(hoje um `while (!isStarted())` dentro de `initialize()`) precisa virar máquina de estado em
`init_loop()`.

### D7 — `isShooting()` sem `getCurrentCommand()`

O Ivy não expõe o comando atual de um recurso. `Robot.isShooting()` passa a ler um
`volatile boolean` setado no `setStart`/`setEnd` do comando de alinhamento, ou
`Scheduler.isRunning(alignCmd)` com o comando guardado em campo. Escolha na Fase 4.

### D8 — Ordem de fases para conter a não-compilação

Fase 0 (deps + delete) → Fase 1 (base nova, isolada) → Fase 2 (subsistemas) → Fase 3 (comandos)
→ Fase 4 (Robot + teleop) → Fase 5 (Autos) → Fase 6 (controladores). Métrica de progresso:
contagem de erros de `:TeamCode:compileDebugJavaWithJavac` decrescente por fase; zero ao fim da
Fase 6.

## Risks / Trade-offs

- **Coordenada Maven do Ivy incerta** (doc: `com.pedropathing:ivy:1.0.0`; repo publica `:core`
  + `:pedro`) → confirmar no primeiro sync da Fase 0 antes de escrever qualquer código;
  registrar a coordenada correta no `MIGRACAO-IVY.md`.
- **Janela de não-compilação nas fases 2–5** → trabalhar no branch `IvyMigrate`, não tocar
  `master`; commit por fase para permitir rollback granular.
- **`Scheduler` global** → esquecer `reset()` vaza comandos entre OpModes; mitigar
  centralizando `reset()` no `RobotOpMode` (`init()` e `stop()`) e testando "rodar → parar →
  rodar" na verificação.
- **Comandos `infinite` sem requirements nunca terminam sozinhos** → auditar cada `periodic()`
  para garantir que é inofensivo entre execuções; `reset()` é a única saída.
- **Máquinas de estado de tiro/mira com estado capturado em lambda** → risco de bug sutil de
  reuso; se ficar frágil, cair para a Class API do Ivy nesses comandos.
- **API do `PIDFController` da Pedro difere** (`updateError`+`run` vs `calculate`) → sinal de
  erro trocado é o erro mais provável; validar cada um dos 3 arquivos com o robô na bancada.
- **Paridade difícil de provar** → o plano de teste no robô (`MIGRACAO-IVY.md §7`) lista as
  checagens observáveis por binding e por rotina; tratá-lo como critério de aceite.

## Migration Plan

1. Branch `IvyMigrate` (já existe). Commit por fase.
2. Fase 0: sync com o Ivy, confirmar coordenada, deletar código morto, baseline de erros.
3. Fases 1–6 conforme D8, `compileDebugJavaWithJavac` ao fechar cada fase.
4. Verificação estática (grep FTCLib vazio, `assembleDebug`) + verificação no robô (bancada).
5. Rollback: reverter para o commit da fase anterior; `master` permanece intocado durante toda
   a migração.

## Open Questions

- Coordenada(s) Maven exata(s) do Ivy 1.0.x — resolvível no sync da Fase 0 sem afetar specs
  nem abordagem.
