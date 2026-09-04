## Why

O código do robô carrega dois frameworks sobrepostos — o command-based da **FTCLib**
(`org.ftclib.ftclib:core:2.1.1`) para subsistemas/comandos e a **Pedro Pathing** para
movimento. A Pedro publicou o **Ivy** (`com.pedropathing.ivy`), um framework command-based
próprio que integra nativamente com o `Follower`. Migrar para o Ivy elimina a sobreposição,
remove a FTCLib por completo e alinha o projeto com a stack que o time já usa. O `master`
hoje nem compila (imports quebrados desde `5be8d9b`), então a migração já entra corrigindo a
base.

## What Changes

- Adicionar a dependência do Ivy e **remover** `org.ftclib.ftclib:core` do `build.dependencies.gradle`. **BREAKING** para qualquer código que importe `com.arcrobotics.ftclib.*`.
- Substituir o `CommandScheduler` (singleton FTCLib) pelo `Scheduler` (estático Ivy): `reset()` no `init()`/`stop()`, `execute()` 1×/loop, sem vazamento de comandos entre execuções.
- Criar `robot/RobotOpMode` — base de OpMode **iterativa** (`extends OpMode`), substituindo `CommandOpMode`.
- Converter os 6 subsistemas vivos: remover `extends SubsystemBase`, expor `Command periodic()` como comando `infinite` de controle contínuo + telemetria.
- Reproduzir o conceito de "default command" (que o Ivy não tem) via comandos `infinite` com `requiring` + prioridade 0 + `InterruptedBehavior.SUSPEND`; comandos de botão usam prioridade ≥ 1.
- Converter os 19 comandos vivos para fábricas `static` que retornam `com.pedropathing.ivy.Command` (`Command.build()` / `Groups` / `Commands`).
- Dividir `robot/RobotContainer` em `robot/Robot` (só fiação); mover os bindings de gamepad para polling no `loop()` do OpMode com os edge-helpers do SDK (`gamepadX.<botao>WasPressed()`), eliminando `GamepadEx`/`GamepadButton`/`Trigger`.
- Reescrever `teleop.java` e `autos/Autos.java` sobre a nova base; a configuração pré-play do seletor migra do loop bloqueante em `initialize()` para máquina de estado em `init_loop()`.
- Trocar `com.arcrobotics.ftclib.controller.PIDFController` por `com.pedropathing.control.PIDFController` nos 3 arquivos vivos que o usam.
- **Deletar** código morto e `@Disabled`: 6 OpModes de auto substituídos pelo seletor, 3 subsistemas-template não usados (Husky/Climber/Elevator) + Constants, `ElevatorTestOpMode`, 5 classes de comando sem referência, `utils/Polygon2d.java`.
- Meta de aceite: **paridade de comportamento** — cada binding, cada rotina de autônomo e cada laço contínuo produz o mesmo efeito observável de hoje.

## Capabilities

### New Capabilities

- `scheduler-lifecycle`: gestão de comandos pelo `Scheduler` do Ivy — reset obrigatório por OpMode, `execute()` por loop, resolução de conflito por requirements/prioridade/`SUSPEND`, ausência de vazamento entre execuções, e a garantia de framework único (zero `com.arcrobotics.ftclib` no código e no gradle).
- `opmode-base`: a base `RobotOpMode` iterativa — ciclo `init` / `init_loop` / `loop` / `stop`, construção do `Robot`, agendamento dos comandos `periodic()`, limpeza do bulk cache dos Lynx hubs 1×/loop.
- `robot-subsystems`: os 6 subsistemas (`Drivetrain`, `Intake`, `Vision`, `Shooter`, `Indexer`, `LED`) como classes simples sem herança de framework, cada uma expondo `Command periodic()` (controle contínuo + telemetria) e fábricas de comando para ações discretas.
- `teleop-control`: o OpMode de teleoperação — comandos contínuos de drive e auto-aim via `SUSPEND`/prioridade, bindings de driver e operador (cada botão → efeito), toggle do auto-aim do shooter, trim de offset de RPM, laço periódico de relocalização por Limelight, restauração de pose inicial.
- `autonomous-routines`: as 3 sequências de autônomo (`rear-normal` com gate, `front` triângulo grande, `rear-no-gate` 15 artefatos) como composições `Groups.sequential` do Ivy, parametrizadas pela lista de `Pose` por aliança, com ciclos de tiro e coletas de linha equivalentes aos scripts atuais.
- `autonomous-selector`: o OpMode `Autos` — configuração pré-play (aliança em X/B, estratégia no D-PAD, `A` confirma) via máquina de estado em `init_loop()`, seleção de rotina + pose inicial por aliança × estratégia, agendamento no `start()`.

### Modified Capabilities

<!-- Nenhuma: openspec/specs/ está vazio; todas as capabilities acima são novas. -->

## Impact

- **Dependências**: `build.dependencies.gradle` — `+ com.pedropathing.ivy` (coordenada a confirmar no sync: doc diz `com.pedropathing:ivy:1.0.0`, repo publica `:core` + `:pedro`), `− org.ftclib.ftclib:core:2.1.1`.
- **Código novo**: `robot/RobotOpMode.java`, `robot/Robot.java`, opcional `autos/AutoRoutines.java`.
- **Reescrito**: `teleop.java`, `autos/Autos.java`, `robot/RobotContainer.java` (→ `Robot`, deletado), 6 subsistemas, 19 comandos.
- **Deletado**: `autos/Auto{Red,Blue}{Front,Rear,Tuff}.java`, `subsystems/templates/{Husky,Climber,Elevator}*.java`, `subsystems/templates/ElevatorTestOpMode.java`, `commands/{AimByPose,ChaseArtifact,TeleOpDriveCommandZoneRepulsion}Command.java`, `autos/commands/{AdjustAuto,AutoChaseArtifact}Command.java`, `utils/Polygon2d.java`.
- **Inalterado**: `autos/paths/*Poses.java` + `PosesNames`, `pedroPathing/Constants.java`, `pedroPathing/Tuning.java`, `pedroPathing/MotorDirections.java`, `utils/DataStorage.java`, `utils/PoseStorage.java`, todos os `*Constants` `@Configurable` (Panels).
- **APIs**: consumidores externos do `RobotContainer` (nenhum fora dos OpModes) e a assinatura dos comandos (`com.arcrobotics.ftclib.command.Command` → `com.pedropathing.ivy.Command`).
- **Risco**: janela de não-compilação entre as fases 2 e 6; `OpMode` iterativo muda o ciclo de vida de `teleop`/`Autos`; `Scheduler` global exige disciplina de `reset()`.
