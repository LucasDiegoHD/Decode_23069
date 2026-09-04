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

A doc oficial diz `implementation 'com.pedropathing:ivy:1.0.0'`. O repositório publica dois
módulos: `com.pedropathing.ivy:core` e `com.pedropathing.ivy:pedro`. Confirmar no primeiro
sync qual(is) coordenada(s) resolve(m) `com.pedropathing.ivy.Scheduler` **e**
`com.pedropathing.ivy.pedro.PedroCommands`. Repo maven já presente no projeto:
`https://maven.pedropathing.com/`. Alternativa: snapshots do Sonatype.

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
- `@Override public void periodic()` → **`public Command periodic()`** retornando
  `Commands.infinite(() -> { <corpo antigo do periodic> })`. **Sem `.requiring(...)`** — roda
  sempre, em paralelo com os comandos de controle.
- Métodos de estado `void` chamados de dentro do `periodic()` / bindings (`intake.run()`,
  `shooter.stop()`, `shooter.adjustRpmOffset()`, `setShootingState()`, …): **mantêm-se**.
- Ações discretas hoje embrulhadas em `new InstantCommand(intake::run, intake)`: adicionar
  fábricas idiomáticas no subsistema — `public Command runIntake()` =
  `Commands.instant(this::run).requiring(intakeMotor)`.

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

Migrar os 6 subsistemas vivos. `DrivetrainSubsystem`: `follower.update()` + `Drawing.*` +
telemetria continuam dentro do `periodic()` infinite; `getFollower()`, `driveRobotCentric()`,
`isRobotStopped()`, `stop()`, `restorePoseFromStorage()`, `getVoltage()` continuam métodos
normais. `ShooterSubsystem`: adiar a troca do `PIDFController` para a Fase 6.

**Entregável:** subsistemas sem herança FTCLib, cada um expondo `Command periodic()`.

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
- `List<Command> periodicCommands()` = `[drivetrain.periodic(), intake.periodic(),
  shooter.periodic(), vision.periodic(), indexer.periodic(), Led.command(led, indexer)]`.
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

- **Coordenada Maven do Ivy** — doc vs. repo divergem; confirmar `core` + `pedro` no primeiro
  sync antes de qualquer código.
- **`OpMode` iterativo vs. `LinearOpMode`** — `teleop` / `Autos` deixam de ter loop
  bloqueante; toda a lógica pré-play de `Autos` precisa virar máquina de estado em
  `init_loop()`.
- **`getCurrentCommand()` não existe no Ivy** — `Robot.isShooting()` precisa de outra
  estratégia (flag ou `Scheduler.isRunning`).
- **Máquinas de estado internas** — `ShootCommand`, `ActiveAimCommand`, `GoToPoseCommand` têm
  estado mutável entre loops; usar array/objeto `final` capturado ou a Class API.
- **`Scheduler` é estático e global** — `Scheduler.reset()` obrigatório no `init()` **e** no
  `stop()` de todo OpMode, senão comandos vazam entre execuções.
- **Comandos `infinite` sem `requiring`** rodam para sempre — só o `Scheduler.reset()` os
  para. Confirmar que todo `infinite` de `periodic()` é inofensivo entre OpModes.
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