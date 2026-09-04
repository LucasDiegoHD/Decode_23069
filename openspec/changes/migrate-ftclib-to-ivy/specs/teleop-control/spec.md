## Purpose

Define o comportamento observável do OpMode de teleoperação: condução e mira contínuas,
mapeamento de botões dos dois controles, e os laços auxiliares de relocalização e pose
inicial.

## ADDED Requirements

### Requirement: Condução e mira contínuas com precedência

Ao iniciar, o teleop SHALL agendar um comando contínuo de condução (reservando o drivetrain,
prioridade base) e um comando contínuo de auto-mira do atirador (reservando o atirador,
prioridade base), ambos com comportamento de suspensão. Comandos de botão que reservam esses
recursos SHALL ter prioridade maior e, ao terminarem, o comando contínuo correspondente SHALL
retomar automaticamente.

#### Scenario: Alinhamento interrompe e devolve a condução

- **WHEN** o piloto segura o botão de alinhar ao AprilTag
- **THEN** a condução manual é suspensa e o robô gira para o alvo
- **WHEN** o piloto solta o botão
- **THEN** a condução manual volta a responder aos analógicos sem novo acionamento

### Requirement: Mapeamento de botões do piloto

O teleop SHALL reproduzir o mapeamento atual do controle do piloto:

| Botão | Ação |
|---|---|
| Y (segurar) | alinhar ao AprilTag enquanto segurado |
| X (segurar) | condução com mira cinemática ao alvo enquanto segurado |
| START | forçar relocalização por Limelight com heading de referência |
| LB (segurar) | rodar o intake enquanto segurado; parar ao soltar |
| RB (segurar) | executar a macro de tiro automático enquanto segurado |

#### Scenario: Botão de segurar liga e desliga

- **WHEN** o piloto pressiona um botão "segurar" e depois o solta
- **THEN** a ação correspondente começa ao pressionar e termina ao soltar

#### Scenario: START relocaliza

- **WHEN** o piloto pressiona START
- **THEN** a pose do robô é redefinida a partir da Limelight com o heading de referência

### Requirement: Mapeamento de botões do operador

O teleop SHALL reproduzir o mapeamento atual do controle do operador:

| Botão | Ação |
|---|---|
| RB (segurar) | macro de tiro automático enquanto segurado |
| LB (segurar) | rodar o intake; parar ao soltar |
| A (segurar) | reverter o intake; parar ao soltar |
| X (segurar) | rodar o motor de gatilho; parar ao soltar |
| D-PAD ▲ | ativar auto-ajuste do atirador |
| D-PAD ▼ | desativar auto-ajuste e parar o atirador |
| D-PAD ▶ | +10 no offset de RPM, com rumble curto |
| D-PAD ◀ | −10 no offset de RPM, com rumble curto |
| botão do analógico esquerdo | zerar o offset de RPM |

#### Scenario: Trim de RPM com feedback tátil

- **WHEN** o operador pressiona D-PAD ▶ ou ◀
- **THEN** o offset de RPM muda em ±10
- **AND** o controle do operador vibra brevemente

#### Scenario: Toggle do auto-ajuste

- **WHEN** o operador pressiona D-PAD ▼
- **THEN** o auto-ajuste é desativado e o atirador para
- **WHEN** o operador pressiona D-PAD ▲
- **THEN** o auto-ajuste volta a ativo

### Requirement: Restauração de pose inicial

Ao iniciar, o teleop SHALL definir a pose do robô a partir da pose persistida (memória entre
OpModes ou arquivo), e, se não houver pose válida, a partir da pose inicial padrão da aliança.

#### Scenario: Teleop retoma de onde o autônomo parou

- **WHEN** um autônomo terminou e persistiu a pose final
- **AND** o teleop é iniciado em seguida
- **THEN** a pose inicial do teleop é a pose final do autônomo

### Requirement: Laço periódico de relocalização por Limelight

O teleop SHALL rodar um laço que, a cada ~1 s, tenta relocalizar por Limelight, exceto quando
o robô está atirando/alinhando ou em movimento.

#### Scenario: Não relocaliza durante o tiro

- **WHEN** o robô está executando o alinhamento para tiro
- **THEN** a relocalização periódica é pulada naquele ciclo
