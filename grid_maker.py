import numpy as np
import matplotlib.pyplot as plt

# Definir os limites da grade
x_min, x_max = -2.0, 2.0
y_min, y_max = -4.0, 4.0

# Escolher uma resolução para a grade
resolution = 0.2  # Tamanho de cada célula na grade

# Calcular o tamanho da grade
x_size = int((x_max - x_min) / resolution) + 1
y_size = int((y_max - y_min) / resolution) + 1

# Inicializar a grade
grid = np.zeros((y_size, x_size))  # Observe a inversão para corresponder a [linha, coluna]

# Função para mapear coordenadas reais para índices da grade
def real_to_grid(x, y):
    x_idx = int((x - x_min) / resolution)
    y_idx = int((y - y_min) / resolution)
    return y_idx, x_idx  # Inverter a ordem para corresponder a [linha, coluna]

# Exemplo de pontos (podem ser início, objetivo, etc.)
points = {
    'start': (0, 0),
    'goal': (1, 1)
}

# Preencher a grade com os pontos
for label, (x, y) in points.items():
    y_idx, x_idx = real_to_grid(x, y)
    print(f"Índice do ponto {label} ({x},{y}) na grade: ({y_idx},{x_idx})")
    grid[y_idx, x_idx] = 1  # Por exemplo, marcando os pontos com '1'

print(grid)

# Verificar visualmente a posição dos pontos na grade
plt.imshow(grid, cmap='gray', origin='upper')  # 'upper' para y para baixo
plt.scatter([x_idx], [y_idx], color='red')  # Destacar o ponto (1,1) em vermelho
plt.title('Grid com pontos destacados')
plt.xlabel('x')
plt.ylabel('y')
plt.gca().invert_yaxis()  # Inverter o eixo y para corresponder ao sistema de coordenadas
plt.show()
