## Purpose

Define o ciclo de vida compartilhado dos OpModes do robô: como o robô é construído, como os
laços contínuos dos subsistemas são iniciados, e o que roda a cada iteração antes da lógica
específica do OpMode.

## ADDED Requirements

### Requirement: Base de OpMode iterativa

Os OpModes de competição (`teleop` e o seletor de autônomo) SHALL usar um ciclo de vida
iterativo com fases distintas de inicialização, laço de pré-início, laço principal e parada —
não um único método bloqueante.

#### Scenario: Fase de pré-início roda sem bloquear

- **WHEN** o OpMode foi inicializado mas o `play` ainda não foi pressionado
- **THEN** a fase de pré-início executa repetidamente
- **AND** entradas de gamepad são lidas e a telemetria é atualizada a cada repetição

### Requirement: Construção do robô na inicialização

Ao inicializar, o OpMode SHALL reinicializar o escalonador, construir uma única instância do
robô (fiação de todos os subsistemas) e agendar os comandos de controle contínuo
(`periodic`) de cada subsistema.

#### Scenario: Subsistemas prontos após o init

- **WHEN** o OpMode termina a inicialização
- **THEN** todos os 6 subsistemas estão construídos e acessíveis pela instância do robô
- **AND** o comando de controle contínuo de cada subsistema está agendado e executando

#### Scenario: Hubs Lynx em cache manual

- **WHEN** o robô é construído
- **THEN** cada Lynx hub está em modo de bulk caching manual

### Requirement: Manutenção por iteração do loop

A cada iteração do laço principal e do laço de pré-início, o OpMode SHALL limpar o bulk cache
dos Lynx hubs uma vez, avançar o escalonador uma vez e atualizar a telemetria.

#### Scenario: Bulk cache limpo uma vez por loop

- **WHEN** uma iteração do laço principal começa
- **THEN** o bulk cache dos hubs é limpo antes de qualquer leitura de sensor daquela iteração

#### Scenario: Escalonador avança uma vez por loop

- **WHEN** uma iteração do laço principal executa
- **THEN** o escalonador é avançado exatamente uma vez naquela iteração

### Requirement: Parada limpa

Ao parar, o OpMode SHALL reinicializar o escalonador para que nenhum comando permaneça ativo.

#### Scenario: Nenhum comando ativo após parar

- **WHEN** o OpMode é parado
- **THEN** o escalonador não tem comandos em execução, suspensos ou na fila
