## Purpose

Define como os comandos do robô são agendados, executados e encerrados durante uma execução
de OpMode, e a garantia de que o projeto usa um único framework command-based sem resíduo da
FTCLib.

## ADDED Requirements

### Requirement: Framework command-based único

O código de `TeamCode` SHALL depender exclusivamente do framework command-based do Ivy para
subsistemas e comandos. O projeto SHALL NOT conter qualquer `import com.arcrobotics.ftclib.*`
e o build SHALL NOT declarar a dependência `org.ftclib.ftclib:core`.

#### Scenario: Nenhum import da FTCLib no código

- **WHEN** se busca por `com.arcrobotics.ftclib` em `TeamCode/src`
- **THEN** não há nenhuma ocorrência

#### Scenario: Dependência removida do build

- **WHEN** se inspeciona `build.dependencies.gradle`
- **THEN** não há linha declarando `org.ftclib.ftclib:core`
- **AND** o projeto compila (`:TeamCode:compileDebugJavaWithJavac`) sem erros

### Requirement: Reset do escalonador por execução de OpMode

Todo OpMode SHALL reinicializar o estado global do escalonador ao iniciar e ao parar, de modo
que nenhum comando agendado por uma execução persista para a execução seguinte.

#### Scenario: Comando não vaza entre execuções

- **WHEN** um OpMode é executado, agenda comandos, e depois é parado
- **AND** o mesmo ou outro OpMode é iniciado em seguida
- **THEN** nenhum comando da execução anterior está ativo
- **AND** nenhum atuador continua acionado por um comando remanescente

### Requirement: Execução cooperativa por loop

O escalonador SHALL avançar todos os comandos ativos exatamente uma vez por iteração do loop
do OpMode, verificando a condição de término de cada um e encerrando os que terminaram.

#### Scenario: Comando de término natural encerra sozinho

- **WHEN** um comando agendado passa a reportar que terminou
- **THEN** o escalonador o encerra e libera os recursos que ele reservava
- **AND** o comando não é mais avançado nas iterações seguintes

#### Scenario: Vários comandos independentes rodam juntos

- **WHEN** múltiplos comandos sem recursos em conflito estão agendados
- **THEN** todos avançam na mesma iteração do loop

### Requirement: Resolução de conflito por recurso e prioridade

Um comando SHALL poder declarar recursos que utiliza. Ao agendar um comando cujo recurso já
está reservado, o escalonador SHALL resolver o conflito por prioridade: um comando de
prioridade maior interrompe o de prioridade menor; um comando de prioridade menor é bloqueado.
Um comando interrompido cujo comportamento é "suspender" SHALL ser retomado automaticamente
quando o recurso voltar a ficar livre, sem reiniciar.

#### Scenario: Comando de maior prioridade assume o recurso

- **WHEN** o comando A (prioridade 0, "suspender") controla o drivetrain
- **AND** o comando B (prioridade 1) que também requer o drivetrain é agendado
- **THEN** A é suspenso e B passa a controlar o drivetrain

#### Scenario: Comando suspenso retoma ao liberar o recurso

- **WHEN** o comando B do cenário anterior termina
- **THEN** o comando A retoma o controle do drivetrain automaticamente
- **AND** A continua de onde estava, sem executar sua rotina de início de novo

#### Scenario: Comando de menor prioridade não interrompe

- **WHEN** um comando de prioridade 0 é agendado enquanto um de prioridade 1 detém o recurso
- **THEN** o comando de prioridade 0 não passa a executar naquele momento
