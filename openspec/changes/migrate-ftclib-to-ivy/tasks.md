## 1. Fase 0 — Dependência e limpeza

- [x] 1.1 Confirmar a coordenada Maven do Ivy fazendo um sync de teste com `com.pedropathing:ivy:1.0.0`; se falhar, com `com.pedropathing.ivy:core` + `com.pedropathing.ivy:pedro`; se ainda falhar, clonar `Pedro-Pathing/Ivy`, rodar `./gradlew deployLocal` e usar `mavenLocal()` + `com.pedropathing:ivy:LOCAL` (fallback do RevAmped). Verificar que `com.pedropathing.ivy.Scheduler` e `com.pedropathing.ivy.pedro.PedroCommands` resolvem, e registrar a coordenada correta em `MIGRACAO-IVY.md §2`
- [x] 1.2 Em `build.dependencies.gradle`, adicionar a dependência confirmada do Ivy e remover `org.ftclib.ftclib:core:2.1.1`; verificar com `./gradlew :TeamCode:dependencies` que o Ivy aparece e a FTCLib não
- [x] 1.3 Deletar os 6 OpModes `@Disabled` `autos/Auto{Red,Blue}{Front,Rear,Tuff}.java`; verificar que nenhum outro arquivo os referencia (`grep -rn "AutoRedRear\|AutoBlueRear\|AutoRedFront\|AutoBlueFront\|AutoRedTuff\|AutoBlueTuff" TeamCode/src` vazio)
- [x] 1.4 Deletar os templates não usados `subsystems/templates/{Husky,Climber,Elevator}Subsystem.java`, seus `*Constants.java` e `ElevatorTestOpMode.java`; verificar `grep -rn "HuskySubsystem\|ClimberSubsystem\|ElevatorSubsystem" TeamCode/src` vazio
- [x] 1.5 Deletar os comandos sem referência `commands/{AimByPose,ChaseArtifact,TeleOpDriveCommandZoneRepulsion}Command.java`, `autos/commands/{AdjustAuto,AutoChaseArtifact}Command.java` e `utils/Polygon2d.java`; verificar que os nomes não aparecem mais em `TeamCode/src`
- [x] 1.6 Remover os imports mortos de `Polygon2d` e `Translation2d` em `robot/RobotContainer.java` e `subsystems/DrivetrainSubsystem.java`; verificar com `grep -n "Polygon2d\|Translation2d"` nesses dois arquivos
- [x] 1.7 Rodar `./gradlew :TeamCode:compileDebugJavaWithJavac` e salvar a saída como baseline de erros da migração (contagem de erros por arquivo)

## 2. Fase 1 — Base de OpMode

- [x] 2.1 Criar `robot/RobotOpMode.java` (`abstract class RobotOpMode extends OpMode`) com `init()` fazendo `Scheduler.reset()`, telemetria Panels, construção do `Robot` e agendamento de um único `Commands.infinite(robot::update)`; verificar que o arquivo compila isoladamente (erros restantes apenas por `Robot` ainda inexistente)
- [x] 2.2 Implementar em `RobotOpMode` os métodos `init_loop()` e `loop()` fazendo `Scheduler.execute()` → `telemetryM.update()`, e `stop()` fazendo `Scheduler.reset()`. O `clearBulkCache()` NÃO vai aqui: pela decisão A2 ele é a primeira linha de `Robot.update()` (tarefa 3.7), que roda dentro do `infinite` agendado — assim o cache é limpo exatamente uma vez por iteração e antes de toda leitura de sensor; verificar por leitura que não há limpeza duplicada

## 3. Fase 2 — Subsistemas

- [x] 3.1 Migrar `subsystems/DrivetrainSubsystem.java`: remover `extends SubsystemBase`, converter `periodic()` em `void update()`, manter `follower.update()`, `Drawing.*`, telemetria e todos os métodos públicos; verificar que a classe compila e que `getFollower()`, `isRobotStopped()`, `driveRobotCentric()`, `stop()`, `restorePoseFromStorage()`, `getVoltage()` seguem presentes
- [x] 3.2 Migrar `subsystems/IntakeSubsystem.java` no mesmo padrão e adicionar fábricas `Command` para ligar, parar, reverter e acionar o gatilho; verificar compilação e que cada fábrica reserva o motor correspondente
- [x] 3.3 Migrar `subsystems/VisionSubsystem.java` no mesmo padrão, preservando as assinaturas de `getRobotPoseMT1/MT2`, `getTargetTx`, `hasTarget`, `getDirectDistanceToTarget`; verificar compilação
- [x] 3.4 Migrar `subsystems/templates/ShooterSubsystem.java` no mesmo padrão, mantendo o `PIDFController` da FTCLib por ora (trocado na Fase 7); verificar compilação e que `isReady()`, `getShooterAtTarget()`, `adjustRpmOffset()`, `resetRpmOffset()`, `stop()` seguem presentes
- [x] 3.5 Migrar `subsystems/templates/IndexerSubsystem.java` no mesmo padrão, preservando a contagem de peças por 3 sensores e o limite de capacidade; verificar compilação
- [x] 3.6 Migrar `subsystems/templates/LEDSubsystem.java` no mesmo padrão, absorvendo o corpo do antigo `LedCommand` (contagem de peças → cor) em `led.update()`; verificar compilação
- [x] 3.7 Criar `robot/Robot.java` com os 6 subsistemas `public final`, construtor `(HardwareMap, TelemetryManager)`, bulk caching manual nos hubs, `clearBulkCache()` e `update()` chamando `clearBulkCache()` e então `drivetrain`, `vision`, `indexer`, `shooter`, `intake`, `led` nessa ordem; verificar que `RobotOpMode` (2.1) passa a compilar contra ele e que o bulk cache é limpo antes de qualquer leitura de sensor do ciclo

## 4. Fase 3 — Comandos

- [x] 4.1 Converter `commands/TeleOpDriveCommand.java` em fábrica `static Command` `infinite` lendo `Gamepad` do SDK, com `.requiring(drivetrain).setPriority(0).setInterruptedBehavior(SUSPEND)`; verificar que a lógica de slew, trava de rumo e escala por tensão foi preservada linha a linha
- [x] 4.2 Converter `commands/ActiveAimCommand.java` em fábrica `infinite` com `.requiring(shooter).setPriority(0).setInterruptedBehavior(SUSPEND)`, com o estado mutável capturado em objeto `final` local ou via Class API; verificar compilação e que a compensação de tempo de voo e movimento lateral foi preservada
- [x] 4.3 Deletar `commands/LedCommand.java` (absorvido por `led.update()` na tarefa 3.6); verificar que o mapeamento contagem→cor sobreviveu idêntico e que nada mais referencia a classe
- [x] 4.4 Converter `commands/AlignToAprilTagCommand.java` em fábrica com `.requiring(drivetrain).setPriority(1)`, expondo uma forma de o `Robot` saber que está ativo (flag em `setStart`/`setEnd`); verificar compilação e preservação da condição de término (setpoint ou 20 loops sem alvo)
- [x] 4.5 Converter `commands/KinematicAimDriveCommand.java` em fábrica com `.requiring(drivetrain).setPriority(1)`; verificar compilação e preservação da predição de movimento
- [x] 4.6 Unificar `commands/ShootCommand.java` e `autos/commands/ShootCommandAutonomous.java` numa fábrica única `shoot(shooter, indexer, intake, IntSupplier n)` com `.requiring(shooter, indexer)`; verificar que os 5 estados da máquina de tiro e seus tempos são idênticos aos atuais
- [x] 4.7 Converter `commands/AutoShootCommand.java` em `Groups.sequential(...)` sobre a fábrica de tiro, migrando só o caminho ativo (não ressuscitar o código comentado); verificar compilação
- [x] 4.8 Unificar `SpinShooterCommand`, `AdjustHoodCommand`, `AdjustShooterCommand`, `AdjustHoodCommandAuto`, `AdjustShooterCommandAuto` em fábricas one-shot via `Commands.instant(...)` com `.requiring(shooter)`; verificar que os polinômios de distância→RPM e distância→capô e o clamp 1000–4500 RPM foram preservados
- [x] 4.9 Converter `commands/UpdatePoseLimelightCommand.java` em fábrica via `Commands.instant(...)` sem requirements, mantendo `forceHardReset` como método `static`; verificar que a fusão ponderada odo/LL, o caso de primeiro boot e a rejeição de salto grande foram preservados
- [x] 4.10 Converter `autos/commands/GoToPoseCommand.java` em builder `GoToPose` com `toCommand()`, construindo o `PathChain` via `Commands.lazy(() -> PedroCommands.follow(follower, chain))` e `.requiring(drivetrain)`; verificar que a API fluente (`setConstraints`, `withMaxPower`, `withNoDeceleration`, `withGlobalDeceleration`, `withTangentHeading`, `withConstantHeading`, `withExitTolerance`) segue disponível
- [x] 4.11 Converter `autos/commands/AlignAndAdjustAutoCommand.java` em `Groups.sequential(adjustShooter, adjustHood)`; verificar compilação

## 5. Fase 4 — Robot e teleop

- [x] 5.1 Completar `robot/Robot.java` (criado em 3.7) com o que resta do antigo `RobotContainer`: poses de início, helpers de Limelight e o campo de auto-ajuste; verificar compilação
- [x] 5.2 Mover para `Robot` o campo `shooterAutoAdjust`, a lógica de pose inicial (`DataStorage.actualPose` → `PoseStorage` → pose padrão da aliança), `setAutoStartPose`, `tryRelocalizeLimelight`, `hasLimelightFix` e `updateRobotPose`; verificar compilação
- [x] 5.3 Implementar `Robot.isShooting()` sem `getCurrentCommand()` (flag do comando de align ou `Scheduler.isRunning`); verificar por leitura que o laço de resync o consulta corretamente
- [x] 5.4 Reescrever `teleop.java` como `extends RobotOpMode`, agendando no `start()` os contínuos de condução e auto-mira e o laço de resync via `Groups.loop(Groups.sequential(waitMs(1000), conditional(...)))`; verificar compilação
- [x] 5.5 Implementar no `loop()` do teleop os bindings do piloto (Y, X, START, LB, RB) por polling com `gamepad1.*WasPressed()/WasReleased()`; verificar contra a tabela em `specs/teleop-control/spec.md` que cada botão tem o efeito especificado
- [x] 5.6 Implementar no `loop()` do teleop os bindings do operador (RB, LB, A, X, D-PAD ▲▼◀▶, botão do analógico esquerdo); verificar contra a tabela em `specs/teleop-control/spec.md`
- [x] 5.7 Preservar no `loop()` do teleop a telemetria de tempo de loop sob `DataStorage.DEBUG_MODE`; verificar que os mesmos campos de antes são publicados
- [x] 5.8 Deletar `robot/RobotContainer.java`; verificar `grep -rn "RobotContainer" TeamCode/src` vazio

## 6. Fase 5 — Autônomo

- [x] 6.1 Converter `AutonomousCommands`, `AutonomousFrontCommands` e `AutonomousTuffCommand` em fábricas `static` (`AutoRoutines.rearNormal/front/rearNoGate(Robot, List<Pose>)`) usando `Groups.sequential`/`Groups.parallel`, traduzindo `withTimeout(t)` para `.raceWith(Commands.waitMs(t))` e removendo o parâmetro `led` não usado; verificar passo a passo que a ordem de comandos é idêntica às versões atuais
- [x] 6.2 Reescrever `autos/Autos.java` como `extends RobotOpMode`, movendo o loop de configuração pré-play de `initialize()` para `init_loop()` como máquina de estado com flag `isConfigured` e edge-helpers do `gamepad2`; verificar compilação
- [x] 6.3 Implementar no `Autos` a seleção de rotina e pose inicial por aliança × estratégia, `robot.setAutoStartPose(startPose)` e persistência de `DataStorage.alliance`; verificar contra `specs/autonomous-selector/spec.md` que as 6 combinações mapeiam para a rotina e a lista de poses corretas
- [x] 6.4 Agendar a rotina escolhida no `start()` do `Autos`, mantendo o `Scheduler.execute()` rodando durante o pré-play para a odometria assentar; verificar por leitura que nada é agendado antes do `play`

## 7. Fase 6 — Controladores

- [x] 7.1 Trocar `com.arcrobotics.ftclib.controller.PIDFController` por `com.pedropathing.control.PIDFController` em `AlignToAprilTagCommand`, ajustando `calculate(measured, setpoint)` para `updateError(setpoint - measured)` + `run()`; verificar compilação e conferir o sinal do erro por leitura
- [x] 7.2 Fazer a mesma troca em `KinematicAimDriveCommand`; verificar compilação e sinal do erro
- [x] 7.3 Fazer a mesma troca em `ShooterSubsystem`, preservando kS/kV, o boost de feedforward de tiro e a compensação de tensão; verificar compilação
- [x] 7.4 Confirmar que `grep -rn "com.arcrobotics.ftclib" TeamCode/src` retorna vazio e que `build.dependencies.gradle` não menciona ftclib

## 8. Verificação

- [x] 8.1 Rodar `./gradlew :TeamCode:compileDebugJavaWithJavac` e confirmar zero erros
- [x] 8.2 Rodar `./gradlew :TeamCode:assembleDebug` e confirmar que o APK é gerado
- [ ] 8.3 No robô com as rodas no ar, rodar `teleop` e confirmar: condução responde aos analógicos; segurar Y suspende a condução e soltar a retoma sem novo acionamento; LB roda o intake; D-PAD ◀/▶ do operador ajusta o offset de RPM com rumble; D-PAD ▲/▼ liga e desliga o auto-ajuste; START relocaliza pela Limelight
- [ ] 8.4 No robô, rodar `Autos` e confirmar: X/B e D-PAD mudam a seleção na telemetria antes do play; A confirma e a telemetria indica pronto; o play dispara a rotina e o robô segue o primeiro path e executa o primeiro ciclo de tiro
- [ ] 8.5 Rodar `teleop` → parar → rodar de novo, e depois `Autos` → parar → `teleop`, confirmando que nenhum comando vaza entre execuções (sem movimento fantasma nem motor ligado) e que a pose persiste do autônomo para o teleop
- [x] 8.6 Atualizar `MIGRACAO-IVY.md` com a coordenada Maven confirmada e quaisquer desvios do plano encontrados durante a implementação
