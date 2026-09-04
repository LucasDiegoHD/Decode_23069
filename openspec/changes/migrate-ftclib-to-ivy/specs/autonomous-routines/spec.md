## Purpose

Define o comportamento observável das rotinas de autônomo do robô — a composição de
movimento, mira e tiro que cada estratégia executa.

## ADDED Requirements

### Requirement: Três rotinas de autônomo

O robô SHALL prover três rotinas de autônomo, cada uma parametrizada pela lista de poses da
aliança:

- **rear-normal**: percurso pela traseira com uso do gate, ~5 ciclos de tiro intercalados com
  coletas de linha.
- **front**: percurso pela frente (triângulo grande).
- **rear-no-gate**: percurso pela traseira sem gate, 5 ciclos de tiro (15 artefatos).

Cada rotina SHALL produzir a mesma sequência de ações observável das rotinas atuais.

#### Scenario: Rotina executa a sequência completa

- **WHEN** uma rotina de autônomo é agendada e o `play` é dado
- **THEN** o robô segue os paths na ordem definida pela lista de poses
- **AND** cada ciclo de tiro ajusta o atirador, alinha ao alvo, espera a velocidade-alvo e
  dispara a contagem de peças definida

#### Scenario: Coleta de linha em paralelo com o intake

- **WHEN** a rotina chega a um trecho de coleta de linha
- **THEN** o robô percorre o trecho enquanto o intake roda simultaneamente

### Requirement: Seleção de poses por aliança

Cada rotina SHALL consumir a lista de poses correspondente à aliança selecionada, indexada
pela ordem do enum de nomes de pose. A ordem do array de poses SHALL permanecer em lockstep
com o enum.

#### Scenario: Aliança vermelha usa poses da vermelha

- **WHEN** a rotina rear-normal é executada para a aliança vermelha
- **THEN** todos os waypoints vêm da lista de poses da traseira vermelha

### Requirement: Encadeamento por composição sequencial

As rotinas SHALL ser construídas por composição de comandos (sequencial e paralelo), sem
depender de subclasses de grupo de comando de framework externo. Os limites de tempo de cada
passo (timeouts) SHALL ser equivalentes aos atuais.

#### Scenario: Passo com timeout encerra no tempo

- **WHEN** um passo de espera por condição não é satisfeito dentro do seu timeout
- **THEN** o passo encerra e a rotina avança para o próximo
