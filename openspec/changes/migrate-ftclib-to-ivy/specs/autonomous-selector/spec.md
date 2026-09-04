## Purpose

Define o comportamento do OpMode seletor de autônomo: como o piloto escolhe aliança e
estratégia antes do início e como a rotina escolhida é preparada e disparada.

## ADDED Requirements

### Requirement: Configuração pré-início

O seletor SHALL permitir escolher aliança e estratégia enquanto o OpMode está inicializado e
antes do `play`, lendo o segundo controle:

- X → aliança azul; B → aliança vermelha
- D-PAD ▲ → estratégia front
- D-PAD ▼ ou ◀ → estratégia rear-normal
- D-PAD ▶ → estratégia rear-no-gate
- A → confirmar a configuração

A telemetria SHALL mostrar a seleção atual e a instrução de confirmar.

#### Scenario: Seleção reflete na telemetria em tempo real

- **WHEN** o piloto pressiona B durante a fase de configuração
- **THEN** a telemetria passa a indicar aliança vermelha imediatamente

#### Scenario: Confirmação encerra a configuração

- **WHEN** o piloto pressiona A
- **THEN** a fase de configuração termina e a telemetria indica que o robô está pronto

### Requirement: Preparação da rotina escolhida

Após a confirmação, o seletor SHALL escolher a rotina e a pose inicial pela combinação
aliança × estratégia, definir a pose inicial do robô e persistir a aliança selecionada para o
teleop.

#### Scenario: Combinação aliança × estratégia

- **WHEN** a configuração confirmada é aliança vermelha + rear-no-gate
- **THEN** a rotina preparada é rear-no-gate com as poses da traseira vermelha
- **AND** a pose inicial do robô é a pose de início dessa lista

### Requirement: Disparo no início

O seletor SHALL agendar a rotina escolhida apenas quando o `play` for pressionado. Durante a
espera pelo `play`, o escalonador SHALL continuar avançando para a odometria assentar.

#### Scenario: Rotina só roda após o play

- **WHEN** a configuração foi confirmada mas o `play` ainda não foi dado
- **THEN** a rotina de autônomo ainda não está agendada
- **WHEN** o `play` é pressionado
- **THEN** a rotina é agendada e começa a executar
