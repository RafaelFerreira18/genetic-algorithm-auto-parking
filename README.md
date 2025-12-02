# Planejamento de Trajetórias para Estacionamento Paralelo

## Descrição
Implementação do algoritmo genético para planejamento de trajetórias de estacionamento paralelo autônomo para robôs móveis do tipo carro (CLMR).

**Artigo Base:** "Trajectory Planning For Car-like Robots Through Curve Parametrization And Genetic Algorithm Optimization With Applications To Autonomous Parking"

**Autores:** Renan P. Vieira, Eduardo V. Argento e Téo C. Revoredo

## Equipe
- [Seu Nome]
- [Outros membros]

## Algoritmo
Combina parametrização polinomial de 5ª ordem com otimização por algoritmo genético para gerar trajetórias suaves e contínuas.

### Parâmetros do Algoritmo Genético (Tabela III do artigo)
- População: 50 indivíduos
- Gerações: 100
- Taxa de Cruzamento: 60%
- Taxa de Mutação: 4%
- Representação: Binária
- Seleção: Roleta
- Domínios: k₀, k₁ ∈ [1, 50], Vs ∈ {-1, 1}

### Parâmetros do Veículo (Tabela II do artigo)
- L = 0.325 m (distância entre eixos)
- A = 0.475 m (comprimento)
- B = 0.29 m (largura)
- φ_max = ±33° (ângulo de esterçamento máximo)
- V_max = 1 m/s
- V̇_max = 0.5 m/s²

## Instruções de Execução

### Requisitos
```bash
pip install numpy matplotlib
```

### Executar
```bash
python genetic_parking.py
```

### Opções
1. Estacionamento Paralelo de Frente
2. Estacionamento Paralelo de Ré
3. Ambos

## Resultados Esperados
O algoritmo deve gerar trajetórias comparáveis aos resultados da Tabela IV do artigo:

**Estacionamento de Frente:**
- k₀ ≈ 1.782, k₁ ≈ 2.316, Vs = 1
- |φ|_max ≈ 19.56°
- Comprimento ≈ 1.476 m

**Estacionamento de Ré:**
- k₀ ≈ 1.00, k₁ ≈ 1.834, Vs = -1
- |φ|_max ≈ 31.94°
- Comprimento ≈ 1.013 m

## Referência
Vieira, R. P., Argento, E. V., & Revoredo, T. C. "Trajectory Planning For Car-like Robots Through Curve Parametrization And Genetic Algorithm Optimization With Applications To Autonomous Parking". IEEE Latin America Transactions.
