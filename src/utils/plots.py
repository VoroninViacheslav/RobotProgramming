import numpy as np
import matplotlib.pyplot as plt

def _plot_angle_comparison(time_array: np.ndarray, actual_angles: np.ndarray, 
                           target_angles: np.ndarray, headless: bool):

    plt.figure(figsize=(10, 6))
    
    actual_deg = np.degrees(actual_angles)
    target_deg = np.degrees(target_angles)
    
    plt.plot(time_array, actual_deg, label='Фактический угол', linewidth=2, color='blue')
    plt.plot(time_array, target_deg, label='Желаемый угол', linewidth=2, 
             linestyle='--', color='red', alpha=0.7)
    
    plt.xlabel('Время (с)', fontsize=12)
    plt.ylabel('Угол маятника (градусы)', fontsize=12)
    plt.title('Сравнение фактического и желаемого угла маятника', fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if headless:
        plot_path = 'angle_comparison.png'
        plt.savefig(plot_path, dpi=150, bbox_inches='tight')
        print(f"\nГрафик сохранен: {plot_path}")
        plt.close()
    else:
        plt.show()


def _plot_control_signal(time_array: np.ndarray, control_signal: np.ndarray, headless: bool):

    plt.figure(figsize=(10, 6))
    
    plt.plot(time_array, control_signal, label='Управляющее воздействие', linewidth=2, color='green')
    plt.axhline(y=0, color='black', linestyle='-', linewidth=0.5, alpha=0.3)
    
    plt.xlabel('Время (с)', fontsize=12)
    plt.ylabel('Управляющее воздействие', fontsize=12)
    plt.title('График управляющего воздействия', fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if headless:
        plot_path = 'control_signal.png'
        plt.savefig(plot_path, dpi=150, bbox_inches='tight')
        print(f"График сохранен: {plot_path}")
        plt.close()
    else:
        plt.show()