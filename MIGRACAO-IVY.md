# Plano de Migração — FTCLib command-based → Pedro Pathing Ivy

> Documento de planejamento. Branch de trabalho: `IvyMigrate`.
> Status: plano aprovado, aguardando quebra em specs.

---

## 1. Contexto

O código do time 23069 organiza subsistemas e comandos com o framework command-based da
**FTCLib** (`org.ftclib.ftclib:core:2.1.1`). A Pedro Pathing — já usada para movimento
(`com.pedropathing:ftc:2.1.2`) — publicou o **Ivy** (`com.pedropathing.ivy`), um framework
command-based próprio, minimalista, que integra nativamente com o `Follower`
(comandos `follow` / `hold` / `turnTo`).

**Objetivo:** migrar toda a estrutura viva de subsistemas e comandos para o Ivy e **remover
a FTCLib por completo**, ficando com um único framework command-based no projeto.

**Resultado esperado:** `teleop` e o seletor `Autos` com comportamento idêntico ao atual,
porém rodando sobre o `Scheduler` do Ivy, sem nenhum `import com.arcrobotics.ftclib.*` no
código e sem a dependência no gradle.

### Motivação

- Um framework só: hoje o projeto carrega FTCLib **e** Pedro Pathing; o Ivy elimina a
  sobreposição.
- Integração nativa com o `Follower` da Pedro (comandos de path prontos, sem `GoToPoseCommand`
  caseiro reinventando `follow`).
- O `master` já não compila (imports quebrados desde `5be8d9b`, que moveu
  `ShooterSubsystem`/`IndexerSubsystem`/`LEDSubsystem` para `subsystems.templates` sem
  atualizar o `RobotContainer`) — a migração já entra corrigindo isso.

### Decisões tomadas (2026-09-03)

| Tema | Decisão |
|---|---|
| Código morto / `@Disabled` | **Deletar.** Não migrar nada que esteja `@Disabled` ou sem referência. |
| FTCLib | **Remover 100%.** Trocar controladores por `com.pedropathing.control.*`, remover `Translation2d`, remover a dependência do gradle. |
| "Default commands" (drive / aim / led) | **Comandos `infinite` com prioridade + `SUSPEND`** (ver §4). |
| OpModes de tuning (`Tuning.java`, `MotorDirections.java`) | Deixar como estão — **não importam FTCLib**, não são afetados. |

---

## 2. Referência da API do Ivy

Fonte: `github.com/Pedro-Pathing/Ivy` (v1.0.0 / `version=1.0.1` no `gradle.properties`) +
`pedropathing.com/docs/ivy`.

O Ivy é **muito mais enxuto** que a FTCLib. **Não existe** classe `Subsystem`, registro de
subsistema, "default command" nem DSL de bindings (`GamepadEx` / `Trigger` / `.whenPressed()`).

| Elemento | API |
|---|---|
| `com.pedropathing.ivy.Command` (interface) | `start()`, `execute()`, `done()`, `end(EndCondition)`, `requirements()`, `priority()`, `interruptedBehavior()`, `conflictBehavior()`, `blockedBehavior()`. Compositores default: `.then(...)`, `.with(...)`, `.raceWith(...)`, `.until(cond)`, `.unless(cond)`, `.proxy()`. Ações: `.schedule()`, `.cancel()`, `.isScheduled()`. Constante `Command.NOOP`. |
| `Command.build()` → `CommandBuilder` | `.setStart(Runnable)`, `.setExecute(Runnable)`, `.setDone(BooleanSupplier)`, `.setEnd(Consumer<EndCondition>)`, `.requiring(Object...)`, `.setPriority(int)`, `.setInterruptedBehavior(...)`, `.setBlockedBehavior(...)`, `.setConflictBehavior(...)`. Todos opcionais; defaults sensatos. |
| `Scheduler` (estático) | `Scheduler.reset()` no init do OpMode; `Scheduler.execute()` 1×/loop; `Scheduler.schedule(cmd...)`; `Scheduler.cancel(cmd)`; `Scheduler.isRunning(cmd)`; `Scheduler.isScheduled(cmd)`. |
| `commands.Commands.*` | `instant(Runnable)`, `waitMs(double)`, `waitUntil(BooleanSupplier)`, `infinite(Runnable)`, `conditional(cond, ifTrue, ifFalse)`, `branch(LinkedHashMap)`, `lazy(Supplier<Command>)`, `match(Supplier<enum>, EnumMap)`, `onInterrupt(Runnable)`. |
| `groups.Groups.*` | `sequential`, `parallel`, `race`, `deadline(deadlineCmd, others...)`, `repeat(cmd, n)`, `loop(cmd)`. |
| `behaviors.*` | `EndCondition{NATURALLY, INTERRUPTED, SUSPENDED}` · `InterruptedBehavior{END, SUSPEND}` · `ConflictBehavior{CANCEL, OVERRIDE, QUEUE}` · `BlockedBehavior{CANCEL, QUEUE}`. |
| `pedro.PedroCommands.*` | `follow(follower, pathChain[, maxPower][, holdEnd])`, `hold(follower[, pose[, constraints]])`, `turnTo(follower, radians[, constraints])`. |

### Comportamento do `Scheduler` relevante para a migração (lido do fonte)

- Comando **sem requirements nunca conflita** → sempre roda em paralelo.
- Ao agendar um comando cujo requirement está ocupado: se o ocupante tem prioridade **menor**,
  é interrompido; se `interruptedBehavior() == SUSPEND`, ele vai para `suspendedCommands`.
- Todo `Scheduler.execute()` tenta **re-ativar** comandos suspensos cujos requirements ficaram
  livres — **sem chamar `start()` de novo**, só volta a chamar `execute()`.
- ⇒ Isto reproduz "default command": o comando contínuo tem prioridade 0 + `SUSPEND`; os
  comandos de botão têm prioridade ≥ 1; quando o de botão termina, o contínuo volta sozinho.

### Coordenada Maven (a confirmar no primeiro sync)

**CONFIRMADA (2026-09-03):**

```gradle
implementation 'com.pedropathing:ivy:1.0.0'
```

Um único `aar` no **Maven Central** (`repo1.maven.org`), já alcançável pelo `mavenCentral()` que
o projeto declara — não precisa de repositório novo. Contém tudo:

```
com.pedropathing.ivy.{Command, CommandBuilder, Scheduler}
com.pedropathing.ivy.behaviors.{BlockedBehavior, ConflictBehavior, EndCondition, InterruptedBehavior}
com.pedropathing.ivy.commands.{Branch, Commands, Conditional, Lazy, Match}
com.pedropathing.ivy.groups.{Deadline, Groups, Loop, Parallel, Race, Repeat, Sequential}
com.pedropathing.ivy.pedro.{Follow, Hold, PedroCommands, Turn}
```

A API do `sources.jar` publicado confere com esta seção: `Scheduler` estático
(`schedule` / `execute` / `reset` / `isRunning` / `isScheduled` / `cancel`) e `Commands` com
`waitMs` / `waitUntil` / `infinite` / `instant` / `conditional` / `branch` / `lazy` / `match` /
`onInterrupt`.

Alternativas que **não** foram necessárias, registradas caso a v1.0.0 seja abandonada:
`com.pedropathing.ivy:core:1.0.1` + `com.pedropathing.ivy:pedro:1.0.1` (os dois módulos que o
`master` do repo publica), ou o fallback do RevAmped (§11): `./gradlew deployLocal` +
`mavenLocal()` + `com.pedropathing:ivy:LOCAL`.

---

## 3. Estado atual (inventário)

### Subsistemas vivos (construídos no `RobotContainer`) — 6

`DrivetrainSubsystem`, `IntakeSubsystem`, `VisionSubsystem` (em `subsystems/`),
`ShooterSubsystem`, `IndexerSubsystem`, `LEDSubsystem` (em `subsystems/templates/`).

Todos `extends SubsystemBase`. Todos pareados com um `*Constants` `@Configurable` (Panels —
**não é FTCLib**, permanece). `DrivetrainSubsystem.java` também contém a classe
package-private `Drawing` (visualização no Panels).

### Comandos vivos — 19

`commands/` (11): `TeleOpDriveCommand`, `ActiveAimCommand`, `LedCommand` (os 3 "default"),
`AlignToAprilTagCommand`, `KinematicAimDriveCommand`, `AutoShootCommand`, `ShootCommand`,
`SpinShooterCommand`, `AdjustHoodCommand`, `AdjustShooterCommand`, `UpdatePoseLimelightCommand`.

`autos/commands/` (8): `GoToPoseCommand`, `ShootCommandAutonomous`, `AlignAndAdjustAutoCommand`,
`AdjustShooterCommandAuto`, `AdjustHoodCommandAuto`, `AutonomousCommands`,
`AutonomousFrontCommands`, `AutonomousTuffCommand`.

### OpModes vivos — 2

`teleop.java` e `autos/Autos.java` (ambos `extends CommandOpMode`).

### Utilitários FTCLib não-comando (a trocar)

- `com.arcrobotics.ftclib.controller.PIDFController` / `PIDController` — em **3 arquivos vivos**:
  `AlignToAprilTagCommand`, `KinematicAimDriveCommand`, `ShooterSubsystem`.
- `com.arcrobotics.ftclib.geometry.Translation2d` — só em **imports não usados**
  (`RobotContainer`, `DrivetrainSubsystem`) + código morto. `utils/Polygon2d.java` usa e só
  serve ao código morto.
- `com.arcrobotics.ftclib.util.MathUtils` — **só em código morto**.

### A DELETAR (nada disso é migrado)

```
autos/AutoBlueFront.java   autos/AutoBlueRear.java   autos/AutoBlueTuff.java
autos/AutoRedFront.java    autos/AutoRedRear.java    autos/AutoRedTuff.java
subsystems/templates/HuskySubsystem.java      subsystems/templates/HuskyConstants.java
subsystems/templates/ClimberSubsystem.java    subsystems/templates/ClimberConstants.java
subsystems/templates/ElevatorSubsystem.java   subsystems/templates/ElevatorConstants.java
subsystems/templates/ElevatorTestOpMode.java
commands/AimByPoseCommand.java                commands/ChaseArtifactCommand.java
commands/TeleOpDriveCommandZoneRepulsion.java
autos/commands/AdjustAutoCommand.java         autos/commands/AutoChaseArtifactCommand.java
utils/Polygon2d.java
```

Após deletar: `grep -rl "com.arcrobotics.ftclib" TeamCode/src` deve listar **somente** os
arquivos vivos a migrar.

---

## 4. Padrões de conversão

### Padrão A — Subsistema

`class XSubsystem extends SubsystemBase` →

- Remover `extends SubsystemBase` e imports FTCLib. Classe simples (mantém
  `@Configurable` / `@Config`).
- Manter o construtor `(HardwareMap, TelemetryManager)`.
- `@Override public void periodic()` → **`public void update()`** com o mesmo corpo. Sem
  `@Override`, sem `Command`. O laço contínuo é **único** e vive no `Robot` (ver Padrão A2).
- Métodos de estado `void` chamados de dentro do `periodic()` / bindings (`intake.run()`,
  `shooter.stop()`, `shooter.adjustRpmOffset()`, `setShootingState()`, …): **mantêm-se**.
- Ações discretas hoje embrulhadas em `new InstantCommand(intake::run, intake)`: adicionar
  fábricas idiomáticas no subsistema — `public Command runIntake()` =
  `Commands.instant(this::run).requiring(intakeMotor)`.

### Padrão A2 — Um único laço contínuo no `Robot`

**Decisão (2026-09-03, após avaliar o RevAmped — §11).** Em vez de cada subsistema expor um
`Command periodic()` agendado separadamente, o `Robot` chama os `update()` em **ordem fixa e
explícita**, e o OpMode agenda **um** `Commands.infinite(robot::update)`:

```java
// Robot.java
public void update() {
    clearBulkCache();
    drivetrain.update();   // follower.update() + Drawing + telemetria
    vision.update();
    indexer.update();
    shooter.update();
    intake.update();
    led.update();          // era o LedCommand
}
```

Motivo: ordem determinística escrita no código (não dependente da ordem de iteração do `Deque`
do `Scheduler`), `clearBulkCache()` garantidamente antes de toda leitura de sensor do ciclo, e
um agendamento em vez de seis.

### Padrão B — "Default command" contínuo

`x.setDefaultCommand(new YCommand(x))` →

- No `start()` do OpMode: `YCommand.build(x).schedule()`, onde o comando é `infinite`,
  `.requiring(x).setPriority(0).setInterruptedBehavior(InterruptedBehavior.SUSPEND)`.
- Comandos de botão que "roubam" o subsistema: `.requiring(x).setPriority(1)` (default
  `interruptedBehavior = END`).
- Quando o comando de botão termina, o `Scheduler` re-ativa o contínuo suspenso sozinho.
- `LedCommand` não conflita com nada → `infinite` **sem `requiring`**, agendado junto dos
  `periodicCommands()`.

### Padrão C — Comando

`class X extends CommandBase` → **classe utilitária com método(s) `static` que retornam
`Command`**, construídos com `Command.build()` / `Groups` / `Commands`.

| FTCLib | Ivy |
|---|---|
| `initialize()` | `.setStart(...)` |
| `execute()` | `.setExecute(...)` |
| `isFinished()` | `.setDone(...)` |
| `end(interrupted)` | `.setEnd(endCondition -> ...)` |
| `addRequirements(x)` | `.requiring(x)` |
| `.withTimeout(ms)` | `.raceWith(Commands.waitMs(ms))` |
| `SequentialCommandGroup` | `Groups.sequential(...)` |
| `ParallelCommandGroup` | `Groups.parallel(...)` |
| `WaitCommand(ms)` | `Commands.waitMs(ms)` |
| `WaitUntilCommand(c)` | `Commands.waitUntil(c)` |
| `ConditionalCommand(a, b, c)` | `Commands.conditional(c, a, b)` |
| `RepeatCommand(c)` | `Groups.loop(c)` |
| `InstantCommand(r)` | `Commands.instant(r)` |

**Estado mutável entre loops** (`ShootCommand`, `ActiveAimCommand`, `GoToPoseCommand`):
capturar em objeto/array `final` local no builder (`int[] state = {0}`) **ou** implementar a
interface `Command` diretamente (Class API do Ivy) para esses casos.

### Padrão D — Bindings

`new GamepadButton(driver, Button.Y).whileHeld(cmd)` (na classe `RobotContainer`) →
**polling no `loop()` do OpMode** com os edge-helpers do SDK 11.2.1
(`gamepad1.yWasPressed()` / `gamepad1.yWasReleased()` — dispensam `GamepadEx`):

```java
if (gamepad1.yWasPressed())  align.schedule();
if (gamepad1.yWasReleased()) align.cancel();
```

---

## 5. Fases de projeto

Ordem pensada para minimizar o tempo em que o projeto não compila. A quebra real de
compilação concentra-se entre a Fase 2 e a Fase 6 (inevitável numa troca de framework);
`:TeamCode:compileDebugJavaWithJavac` é a checagem principal ao fechar a Fase 6.

### Fase 0 — Dependência + limpeza

1. `build.dependencies.gradle`: remover `implementation 'org.ftclib.ftclib:core:2.1.1'`;
   adicionar o(s) artefato(s) do Ivy (coordenada confirmada no sync).
2. Deletar todos os arquivos da lista "A DELETAR" (§3).
3. Remover imports mortos de `Polygon2d` / `Translation2d` em `RobotContainer.java` e
   `DrivetrainSubsystem.java`.
4. `./gradlew :TeamCode:compileDebugJavaWithJavac` — falha esperada nos arquivos vivos; serve
   de baseline.

**Entregável:** dependência trocada, código morto removido, escopo real da migração visível
via `grep`.

### Fase 1 — OpMode base

Criar `robot/RobotOpMode.java` — `abstract class RobotOpMode extends OpMode` (iterativo, não
`LinearOpMode` / `CommandOpMode`):

```java
public abstract class RobotOpMode extends OpMode {
    protected Robot robot;
    protected TelemetryManager telemetryM;

    @Override public void init() {
        Scheduler.reset();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Robot(hardwareMap, telemetryM);
        Scheduler.schedule(robot.periodicCommands());
    }
    @Override public void init_loop() { robot.clearBulkCache(); Scheduler.execute(); telemetryM.update(); }
    @Override public void loop()      { robot.clearBulkCache(); Scheduler.execute(); telemetryM.update(); }
    @Override public void stop()      { Scheduler.reset(); }
}
```

**Entregável:** base de OpMode compilando isoladamente.

### Fase 2 — Subsistemas (Padrão A)

Migrar os 6 subsistemas vivos (Padrão A) e montar o laço único (Padrão A2).
`DrivetrainSubsystem`: `follower.update()` + `Drawing.*` + telemetria vão para o `update()`;
`getFollower()`, `driveRobotCentric()`, `isRobotStopped()`, `stop()`,
`restorePoseFromStorage()`, `getVoltage()` continuam métodos normais. `LEDSubsystem`: o corpo
do antigo `LedCommand` vira `led.update()` — deixa de existir como comando.
`ShooterSubsystem`: adiar a troca do `PIDFController` para a Fase 6.

**Entregável:** subsistemas sem herança FTCLib, cada um expondo `void update()`, e
`Robot.update()` chamando-os em ordem fixa.

### Fase 3 — Comandos (Padrão C)

| Comando atual | Vira | Observações |
|---|---|---|
| `TeleOpDriveCommand(drivetrain, driver)` | `teleOpDrive(DrivetrainSubsystem, Gamepad)` — `infinite`, prioridade 0, `SUSPEND` | lê `Gamepad` do SDK direto; slew / heading-hold / voltage no `setExecute` |
| `ActiveAimCommand(...)` | `infinite`, `.requiring(shooter)`, prioridade 0, `SUSPEND` | supplier `isShooterAutoAdjustActive` vira campo no `Robot` |
| `LedCommand(led, indexer)` | `infinite`, **sem requiring** | agendado nos `periodicCommands()` |
| `AlignToAprilTagCommand` | `alignToAprilTag(...)` — `.requiring(drivetrain).setPriority(1)` | trocar `PIDFController` (Fase 6) |
| `KinematicAimDriveCommand` | `.requiring(drivetrain).setPriority(1)` | idem controlador |
| `AutoShootCommand` | `autoShoot(...)` = `Groups.sequential(...)` | hoje só embrulha `ShootCommand` |
| `ShootCommand` / `ShootCommandAutonomous` | fábrica única `shoot(shooter, indexer, intake, IntSupplier n)` | máquina de 5 estados; `.requiring(shooter, indexer)`; unifica as duas versões |
| `SpinShooterCommand` / `AdjustHood*` / `AdjustShooter*` | fábricas one-shot via `Commands.instant(...)` | `.requiring(shooter)`; unifica auto / teleop |
| `UpdatePoseLimelightCommand` | `updatePoseLimelight(...)` via `instant(...)` + `static forceHardReset(...)` mantido | sem requirements |
| `GoToPoseCommand` | classe `GoToPose` **builder** (mantém API fluente) com `toCommand()` | `PathChain` da pose atual embrulhado em `Commands.lazy(() -> PedroCommands.follow(follower, buildChain()))`; `.setDone(() -> !follower.isBusy())`; `.requiring(drivetrain)` |
| `AlignAndAdjustAutoCommand` | `Groups.sequential(adjustShooter, adjustHood)` | |
| `AutonomousCommands` / `FrontCommands` / `TuffCommand` | fábricas `static rearNormal / front / rearNoGate(Robot, List<Pose>)` = `Groups.sequential(...)` | traduzir `addCommands(...)` 1:1; parâmetro `led` não usado — remover |

Constraints de path em `pedroPathing/Constants.java` (`autoTransitConstraints`,
`autoShootConstraints`) **não mudam**.

**Entregável:** todos os comandos vivos como fábricas Ivy.

### Fase 4 — `RobotContainer` → `Robot` + bindings

`RobotContainer.java` (279 linhas: fiação + poses + resync + bindings + getters de auto)
**divide** em:

**`robot/Robot.java`** — só fiação:
- Campos `public final` dos 6 subsistemas; construtor `(HardwareMap, TelemetryManager)`.
- `setBulkCachingMode(MANUAL)` nos hubs + `clearBulkCache()`.
- `void update()` = `clearBulkCache()` seguido de `drivetrain.update()`, `vision.update()`,
  `indexer.update()`, `shooter.update()`, `intake.update()`, `led.update()` **nessa ordem**
  (Padrão A2). O `RobotOpMode` agenda um único `Commands.infinite(robot::update)`.
- Campo `public boolean shooterAutoAdjust = true`.
- `boolean isShooting()` — hoje usa `drivetrain.getCurrentCommand() instanceof
  AlignToAprilTagCommand` (não existe no Ivy). Trocar por `volatile boolean` setado no
  `setStart` / `setEnd` do comando de align, **ou** `Scheduler.isRunning(alignCmd)` com o
  comando guardado num campo.
- Mantidos: `setAutoStartPose`, `tryRelocalizeLimelight`, `hasLimelightFix`, lógica de
  start-pose (`DataStorage.actualPose` / `PoseStorage`), `updateRobotPose`.

**`teleop.java`** — reescrito `extends RobotOpMode`:
- `start()`: aplicar start-pose; agendar `teleOpDrive` e `activeAim`; agendar o loop de resync
  de Limelight (hoje `periodicUpdateLoop`) como
  `Groups.loop(Groups.sequential(Commands.waitMs(1000), Commands.conditional(() ->
  robot.isShooting() || !robot.drivetrain.isRobotStopped(), Command.NOOP,
  updatePoseLimelight(...))))`.
- `loop()`: polling dos bindings (Padrão D) — traduzir a tabela driver / operator atual;
  manter a telemetria de loop-time (`DataStorage.DEBUG_MODE`).

**Entregável:** `Robot` puro + `teleop` funcional sobre o `Scheduler`.

### Fase 5 — `Autos.java`

`extends RobotOpMode` (iterativo). O loop de configuração pré-play que hoje está dentro de
`initialize()` (bloqueante, estilo `LinearOpMode`) **move para `init_loop()`** com máquina de
estado por flags (`isConfigured`), usando os edge-helpers do `gamepad2`. Ao confirmar (`A`):
escolher a fábrica de auto + `startPose` por aliança × estratégia,
`robot.setAutoStartPose(startPose)`, `DataStorage.alliance = ...`. No `start()`:
`cmd.schedule()`. Os 6 getters `getAutonomous*Command()` viram chamadas às fábricas da Fase 3.

**Entregável:** seletor de autônomo funcional sobre o `Scheduler`.

### Fase 6 — Controladores FTCLib → Pedro

Nos 3 arquivos vivos (`AlignToAprilTagCommand`, `KinematicAimDriveCommand`,
`ShooterSubsystem`): trocar `com.arcrobotics.ftclib.controller.PIDFController` por
`com.pedropathing.control.PIDFController`.

- FTCLib: `new PIDFController(kp, ki, kd, kf)` + `calculate(measured, setpoint)`.
- Pedro: `new PIDFController(new PIDFCoefficients(kp, ki, kd, kf))` +
  `updateError(setpoint - measured)` + `run()` (padrão já usado no `Drivetrain.java` do repo
  de exemplo do Ivy). Ajustar chamadas e sinais de erro.

**Entregável:** `grep -rn "com.arcrobotics.ftclib" TeamCode/src` **vazio**; dependência
removida do gradle.

---

## 6. Análise de implementação — pontos de atenção

- **Coordenada Maven do Ivy** — doc vs. repo divergem. §2 tem a ordem de tentativa e o
  fallback comprovado (`deployLocal` + `mavenLocal()`), então é risco de tempo, não de bloqueio.
- **Não copiar formas de API do RevAmped** (§11) — eles rodam um Ivy pré-1.0 (`ICommand`,
  `Scheduler.getInstance()`, `new Instant(...)`, `new Sequential(...)`). A API 1.0.0 é
  `Command`, `Scheduler` estático, `Commands.instant(...)`, `Groups.sequential(...)`.
- **`OpMode` iterativo vs. `LinearOpMode`** — `teleop` / `Autos` deixam de ter loop
  bloqueante; toda a lógica pré-play de `Autos` precisa virar máquina de estado em
  `init_loop()`.
- **`getCurrentCommand()` não existe no Ivy** — `Robot.isShooting()` precisa de outra
  estratégia (flag ou `Scheduler.isRunning`).
- **Máquinas de estado internas** — `ShootCommand`, `ActiveAimCommand`, `GoToPoseCommand` têm
  estado mutável entre loops; usar array/objeto `final` capturado ou a Class API.
- **`Scheduler` é estático e global** — `Scheduler.reset()` obrigatório no `init()` **e** no
  `stop()` de todo OpMode, senão comandos vazam entre execuções.
- **O `infinite(robot::update)` roda para sempre** — só o `Scheduler.reset()` o para.
  Confirmar que é inofensivo entre OpModes.
- **`AutoShootCommand` / `AlignAndAdjustAutoCommand`** já têm muita coisa comentada — migrar
  só o caminho ativo, não ressuscitar o comentado.
- **Bugs pré-existentes achados no inventário** (não corrigir agora, salvo se atrapalharem):
  ternários de aliança aparentemente trocados em `RobotContainer` (`endPose` /
  `innitialPose`); `IntakeConstants` efetivamente morto (o subsistema hard-coda os nomes dos
  motores); `import org.opencv.core.Mat` perdido em `RedRearPoses`. Anotar; decidir com o time
  depois.

---

## 7. Plano de teste

### Por fase (checagem de build)

| Fase | Checagem |
|---|---|
| 0 | `./gradlew :TeamCode:compileDebugJavaWithJavac` — falha esperada, registrar baseline de erros |
| 1 | compila a nova base isoladamente (ainda com erros nos arquivos não migrados) |
| 2–5 | contagem de erros de compilação **decrescente** a cada fase |
| 6 | `./gradlew :TeamCode:compileDebugJavaWithJavac` **limpo** |

### Verificação final (estática)

1. `./gradlew :TeamCode:compileDebugJavaWithJavac` sem erros.
2. `grep -rn "com.arcrobotics.ftclib" TeamCode/src` → vazio.
3. `grep -rni "ftclib" *.gradle` → vazio.
4. `./gradlew :TeamCode:assembleDebug` gera o APK.

### Verificação final (no robô — bancada, rodas no ar)

**`teleop`:**
- Drive responde aos analógicos.
- Segurar **Y** (align to AprilTag) **interrompe** o drive; ao **soltar**, o drive **volta
  sozinho** — prova do `SUSPEND` / resume do `Scheduler`.
- **LB** roda o intake; soltar para.
- **DPAD ◀ / ▶** do operador ajusta o offset de RPM + rumble.
- **DPAD ▲ / ▼** do operador liga / desliga o auto-aim do shooter.
- **START** do driver força relocalização por Limelight.
- Telemetria de loop-time aparece com `DataStorage.DEBUG_MODE`.

**`Autos` (seletor):**
- Configurar aliança (X / B) e estratégia (D-PAD) **antes** do play; telemetria reflete a
  seleção em tempo real.
- **A** confirma; telemetria mostra "PRONTO".
- `play` dispara a sequência; o robô segue o primeiro path e o primeiro ciclo de tiro executa.

**Higiene do `Scheduler`:**
- Rodar `teleop`, parar, rodar de novo — **nenhum** comando "vazado" do run anterior
  (movimento fantasma, motor ligado). Repetir com `Autos`.
- Alternar `Autos` → `teleop` — a pose persiste (`DataStorage` / `PoseStorage`) e o teleop
  retoma de onde o auto parou.

### Regressão de comportamento

O critério de aceite é **paridade**: cada binding, cada sequência de autônomo e cada laço
contínuo do sistema atual tem o mesmo efeito observável depois da migração. Qualquer
diferença é bug de migração, não "melhoria".

---

## 8. Arquivos afetados (resumo)

| Arquivo | Ação |
|---|---|
| `build.dependencies.gradle` | − FTCLib, + Ivy |
| `robot/RobotContainer.java` | dividir em `robot/Robot.java`; deletar o original |
| `robot/RobotOpMode.java` | **novo** — base de OpMode iterativa |
| `teleop.java` | reescrever — bindings via polling no `loop()` |
| `autos/Autos.java` | reescrever — config em `init_loop()` |
| `subsystems/**/*.java` (6) | Padrão A |
| `commands/*.java` (11) + `autos/commands/*.java` (8) | Padrão C |
| `autos/commands/Autonomous*.java` | fábricas `static` (nova `autos/AutoRoutines.java` opcional) |
| `autos/paths/*Poses.java`, `PosesNames.java` | **inalterados** (já usam `com.pedropathing.geometry.Pose`) |
| `utils/DataStorage.java`, `utils/PoseStorage.java` | inalterados |
| `pedroPathing/Constants.java`, `Tuning.java`, `MotorDirections.java` | inalterados |
| lista "A DELETAR" (§3) | deletados |

---

## 9. Resposta honesta: os ganhos são **moderados**, não transformadores. Vale a pena principalmente se você valoriza consolidação e integração com a Pedro.

**Ganhos reais:**

- **Um framework só.** Hoje o projeto carrega FTCLib *e* Pedro Pathing. O Ivy elimina a sobreposição — menos dependências, menos "qual PIDFController é esse?", menos superfície pra manter.
- **Integração nativa com o Follower.** `PedroCommands.follow/hold/turnTo` já vêm prontos. O `GoToPoseCommand` de vocês (o maior comando de autônomo) hoje reimplementa à mão o que o Ivy dá de graça.
- **Scheduler mais simples e previsível.** ~200 linhas legíveis vs. a caixa-preta da FTCLib. O modelo de `requirements` + `priority` + `SUSPEND/resume` é mais explícito que os "default commands" mágicos.
- **Força uma faxina.** A migração já deleta ~13 arquivos mortos, conserta o `master` quebrado e tira imports podres. Parte do "ganho" é isso, e podia ser feito sem migrar.
- **Alinhamento com a comunidade.** Os times de referência da Pedro na DECODE (MOE, Code Blooded, Traffic Cones) já usam Ivy — mais exemplos e suporte no Discord.

**Custos / riscos:**

- É **grande**: ~25 arquivos, 6 subsistemas, 19 comandos, os 2 OpModes principais reescritos. Robô de competição.
- O Ivy é **novo** (v1.0.0, coordenada Maven ainda incerta). Menos maduro que a FTCLib.
- Você **perde conveniências**: `GamepadEx`, DSL de bindings (`.whenPressed()`), `SubsystemBase.periodic()` automático. Vira polling manual e agendamento explícito — mais verboso em alguns pontos.
- O critério de sucesso é **paridade** — muito trabalho pra chegar no mesmo comportamento observável. Nenhuma capacidade nova de robô sai disso.

**Recomendação:** se o time tem tempo de bancada antes do próximo evento e alguém quer ser dono do código de framework, faça — vocês saem com uma base mais limpa e integrada. Se a agenda está apertada, o retorno não justifica o risco agora; dá pra capturar 60% do valor só deletando o código morto e consertando os imports do `master`, sem trocar de framework.
---

## 10. Anexo — avaliação do `kleongf/FTC_Decode` (time 23641)

Repo de referência em autônomo (`github.com/kleongf/FTC_Decode`, DECODE 2025-26).
**Conclusão: não muda o escopo desta migração** — valida decisões de ciclo de vida e gera
quatro melhorias ortogonais, registradas como trabalho futuro.

### 10.1 Correção de premissa

O 23641 **não usa FTCLib**. O `build.dependencies.gradle` deles tem SDK 11.0.0 +
`com.pedropathing:ftc:2.1.0` + `telemetry` + Panels + Dashboard. Zero `org.ftclib`.

E **não usa command-based**. O `lib/robot/{Robot,Subsystem,Command}.java` são três classes
abstratas de ~10 linhas cada. `Command` tem um único método, `build()`, que devolve um
`StateMachine`. Não há scheduler, requirements, prioridade nem composição de comandos.

### 10.2 A arquitetura deles

**Subsistema = campos públicos de estado desejado + `update()`.** `intake.wantedMode`,
`shooter.wantedVelocity` / `wantedPitch`, `turret.wantedAngle`. Quem quer agir **escreve no
campo**; o `update()` do subsistema aplica no hardware. É o padrão "want state" (estilo FRC
254). Ninguém reserva subsistema — daí não precisarem de resolução de conflito.

**`CurrentRobot.update()` é um laço plano e determinístico:**

```java
dt = loopTimer.seconds(); loopTimer.reset();
bulkRead.clearCache();
for (Subsystem s : subsystems) s.update();
for (StateMachine c : commands) c.update();
```

**"Comandos" = `StateMachine` pré-construídos** uma vez em `registerCommands()` e
re-`start()`ados sob demanda. Ficam sempre na lista de update; quando terminam, viram inertes.

**Autos = um `StateMachine` gigante** de `State` com `onEnter` / `onExit` / `minTime` /
`maxTime` / `transition(cond)` / `fallbackState`, montado no `init()`, `.start()` no `start()`,
`.update()` no `loop()`. OpModes são iterativos (`extends OpMode`).

### 10.3 O que valida do nosso plano

| Nossa decisão | 23641 |
|---|---|
| Fase 1 — `OpMode` iterativo, não `LinearOpMode` / `CommandOpMode` | idêntico |
| Fase 5 — configuração pré-play em `init_loop()` | idêntico (`gamepad1.dpadRightWasPressed()` no `init_loop`) |
| Padrão D — bindings por polling com `WasPressed()` | idêntico; teleop inteiro assim |
| Padrão A — controle contínuo por subsistema | é o `update()` deles |

### 10.4 Por que o FSM deles não substitui o `Groups` do Ivy

| FSM (23641) | Ivy |
|---|---|
| `.onEnter(r)` / `.onExit(r)` | `.setStart(r)` / `.setEnd(r)` |
| `.maxTime(t)` | `.raceWith(Commands.waitMs(t))` |
| `.minTime(t)` | `Groups.sequential(Commands.waitMs(t), ...)` |
| `.transition(cond)` | `Commands.waitUntil(cond)` |
| `fallbackState` / branch | `Commands.conditional` / `branch` / `match` |

O Ivy expressa tudo. Não importar o FSM deles.

### 10.5 O que NÃO copiar

- Camada de "command" vestigial; `Subsystem` duplicado em `lib/robot` **e** `util/decodeutil`;
  comentários do tipo `// todo: adapt to this or something lol`. É código de competição, não
  framework.
- Autos com paths hardcoded por OpMode — 8 arquivos, `RedClose24.java` com 538 linhas.
- A ausência de resolução de conflito só funciona porque o drive em teleop é chamada direta.
  Nós temos `align` e `kinematicAimDrive` disputando o drivetrain, então o mecanismo de
  prioridade + `SUSPEND` (§4, Padrão B) continua necessário.

### 10.6 Trabalho futuro registrado (fora desta migração)

Decisão: **apenas registrar**. Não entram no escopo da migração — aumentariam a janela de
não-compilação e quebrariam o critério de aceite de paridade (§7). Abrir change(s) OpenSpec
separadas depois.

1. **`blackboard` do SDK para handoff de pose entre OpModes.** Eles fazem
   `blackboard.put(END_POSE_KEY, follower.getPose())` no `stop()` do auto e leem no teleop.
   Substituiria `utils/PoseStorage.java` (arquivo em `/sdcard/stored_robot_pose.txt`) e
   `DataStorage.actualPose`. Duas classes a menos, sem I/O de arquivo.
2. **`Flipper` / `Mirrorer` — um auto, duas alianças por espelhamento.** Hoje mantemos quatro
   `autos/paths/*Poses.java` em lockstep com `PosesNames` por ordinal, o que é frágil por
   construção.
3. **Caching de potência de motor** — `if (|prevSetPower − wanted| > threshold) setPower(...)`
   em `Intake` / `Shooter`. Corta escritas redundantes; ganho de loop-time.
4. **`SOTMUtil` unificado** — uma classe que, de pose + velocidade + aceleração, devolve
   `{turretAngle, hoodAngle, wheelVelocity, feedforwards}`. Nossa lógica equivalente está
   espalhada entre `ActiveAimCommand` e `KinematicAimDriveCommand`.

---

## 11. Anexo — avaliação do `junkjunk123/RevAmped-Decode-V2` (time 12808, vencedor do MTI)

Time que **usa Ivy** e conta com um dos desenvolvedores do Pedro Pathing. O uso arquitetural é
autoritativo — é como um autor da biblioteca a usa. As formas de API, não (ver §11.1).

### 11.1 Rodam um Ivy pré-1.0 — não copiar formas de API

```gradle
mavenLocal()
implementation 'com.pedropathing:ivy:LOCAL'
```

Compilam o Ivy do fonte, a partir do working copy do próprio dev. O código usa `ICommand`,
`Scheduler.getInstance()` (singleton), `new Instant(...)`, `new Sequential(...)`,
`new Command().setStart(...)`. A API 1.0.0 publicada usa `Command` (interface), `Scheduler`
**estático**, `Commands.instant(...)`, `Groups.sequential(...)`, `Command.build()`.

Consequências:

- **Traduzir, nunca copiar** trechos do código deles.
- **Confirma o risco de coordenada Maven** e fornece o fallback (`deployLocal` +
  `mavenLocal()`), já incorporado a §2 e à Fase 0.

### 11.2 ADOTADO — um único `Infinite`, não um por subsistema

```java
schedule(new Infinite(() -> { robot.update(); autoTrack.update(); telemetry.update(); }));
```

`Robot.update()` faz `clearBulkCache()` e chama cada mecanismo em ordem fixa. Adotado como
**Padrão A2** (§4), substituindo a proposta original de seis comandos `periodic()`.

### 11.3 `PathSupplier` — a lição de autos (trabalho futuro)

`PathSupplier` fornece uma `List<FollowParameters>` em ordem; `drivetrain.follow()`
**desenfileira o próximo**. Os autos viram composição de **métodos de ciclo reutilizáveis**:

```java
robot.drivetrain.follow(), shoot(),
sideSpikeCycle(), spikeCycle(),
gateCycle(), gateCycle(), gateCycle(), gateCycle(), gateCycle(), gateCycle(),
shootLast()
```

`FollowParameters` é um `record (pathChain, holdEnd, maxPower, kP, brakingStrength)` — encapsula
os parâmetros de seguimento por trecho, incluindo os coeficientes de frenagem preditiva.

Nosso equivalente: `Pose[] POSES` indexado por ordinal de enum, em quatro arquivos que precisam
ficar em lockstep, com o ciclo de tiro repetido inline cinco vezes dentro de
`AutonomousCommands`. A diferença de manutenibilidade é grande. **Decisão: trabalho futuro** —
não entra na migração, que quebraria o critério de paridade (§7).

### 11.4 `TeleOpStateHandler` — grafo de estados (trabalho futuro)

Estado do robô (`INTAKE → DRIVE_TO_SHOOT → SHOOT`) com transições validadas contra uma **matriz
de adjacência**, exatamente uma transição por vez, no máximo uma enfileirada, e um contador de
abort. É o que impede o teleop de quebrar quando o piloto martela botões. Nosso
`isShooterAutoAdjustActive` + `isShooting()` é a versão primitiva do mesmo problema.

### 11.5 `BooleanSwitch` + `ButtonMapper` (avaliar só se necessário)

Cerca de 130 linhas que devolvem a DSL de bindings perdida ao largar o `GamepadEx`:
`risingEdge()`, `fallingEdge()`, `toggled()`, `and()`, `or()`, debounce por timestamp.
**Ressalva:** no `MTITele` real eles quase não usam o `ButtonMapper` — chamam
`gamepad_1.right_bumper.isRisingEdge()` direto. Nosso polling cru (§4, Padrão D) basta; só vale
importar o `toggled()` se aparecer necessidade de toggle.

### 11.6 O que NÃO copiar

- `Robot.INSTANCE` estático (singleton global).
- `TrackingThread` / `GyroThread` — threads de tracking; e no auto eles nem rodam threaded,
  chamam `autoTrack.update()` dentro do `Infinite`.
- `OpModeCommand extends LinearOpMode` com skeleton `runOpMode()` manual. Nossa base iterativa
  (Fase 1) é mais simples e é o que os outros dois repos de referência fazem.

### 11.7 Comparação dos três repos de referência

| | 23641 (kleongf) | 12808 (RevAmped) | 23069 (nosso plano) |
|---|---|---|---|
| Framework | nenhum (FSM próprio) | Ivy pré-1.0 | Ivy 1.0.0 |
| OpMode | iterativo | `LinearOpMode` + skeleton | iterativo |
| Laço contínuo | `robot.update()` plano | **1 `Infinite`** | **1 `Infinite`** |
| Bindings | polling `WasPressed()` | `BooleanSwitch` edges | polling `WasPressed()` |
| Autos | FSM gigante, paths inline | fila de paths + ciclos | `Groups.sequential` (paridade) |
| Estado do robô | campos "wanted" | grafo com matriz | flags (como hoje) |

Os três convergem em: **um laço contínuo determinístico**, **bindings por polling com detecção
de borda** e **configuração pré-play num loop de init**. Nenhum dos três usa a FTCLib.

---

## 12. Diário de implementação

### Fase 0 — concluída (2026-09-03)

- **Coordenada Maven resolvida na primeira tentativa.** `com.pedropathing:ivy:1.0.0` está no
  Maven Central e o `mavenCentral()` que o projeto já declara basta — nem o repositório da
  Pedro nem o fallback `deployLocal` foram necessários. Detalhes em §2.
- FTCLib removida do `build.dependencies.gradle`; `:TeamCode:dependencies` confirma o Ivy no
  `debugCompileClasspath` e nenhum `org.ftclib`.
- 19 arquivos deletados (6 autos `@Disabled`, 3 subsistemas-template + Constants,
  `ElevatorTestOpMode`, 5 comandos sem referência, `utils/Polygon2d.java`).
- Imports mortos de `Translation2d` / `Polygon2d` e as linhas comentadas do Husky removidas de
  `RobotContainer.java` e `DrivetrainSubsystem.java`.
- **Baseline de compilação: 339 erros em 28 arquivos**, todos por ausência da FTCLib. Esta é a
  métrica de progresso das fases seguintes — deve chegar a zero na Fase 6.

  > **Armadilha:** o `javac` trunca em 100 erros por padrão. Para medir o baseline de verdade,
  > compile com `-Xmaxerrs` elevado, via init script:
  > `./gradlew -I maxerrs.gradle :TeamCode:compileDebugJavaWithJavac`, com
  > `allprojects { tasks.withType(JavaCompile).configureEach { options.compilerArgs << "-Xmaxerrs" << "10000" } }`.

| Erros | Arquivo | Erros | Arquivo |
|---:|---|---:|---|
| 79 | `RobotContainer.java` | 5 | `ActiveAimCommand.java` |
| 39 | `Autos.java` | 5 | `AutoShootCommand.java` |
| 28 | `AutonomousCommands.java` | 5 | `SpinShooterCommand.java` |
| 26 | `AutonomousFrontCommands.java` | 4 | `AdjustHoodCommandAuto.java` |
| 26 | `AutonomousTuffCommand.java` | 4 | `AdjustShooterCommandAuto.java` |
| 18 | `teleop.java` | 4 | `AlignAndAdjustAutoCommand.java` |
| 13 | `AlignToAprilTagCommand.java` | 4 | `AdjustHoodCommand.java` |
| 12 | `KinematicAimDriveCommand.java` | 4 | `AdjustShooterCommand.java` |
| 11 | `ShootCommand.java` | 4 | `UpdatePoseLimelightCommand.java` |
| 8 | `TeleOpDriveCommand.java` | 3 | `DrivetrainSubsystem.java` |
| 7 | `ShootCommandAutonomous.java` | 3 | `IntakeSubsystem.java` |
| 6 | `ShooterSubsystem.java` | 3 | `VisionSubsystem.java` |
| 6 | `GoToPoseCommand.java` | 3 | `IndexerSubsystem.java` |
| 6 | `LedCommand.java` | 3 | `LEDSubsystem.java` |

**Nota de ambiente:** o build exige `local.properties` com `sdk.dir` (gitignored) e um JDK 17–21
— o `jbr` do Android Studio (JDK 21) apontado por `JAVA_HOME` serve; o `jdk-25` que está no
`PATH` não é suportado pelo Gradle 8.9.
