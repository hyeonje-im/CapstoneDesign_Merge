from Pages.Grid_visualization import GridVisualization

# 전체 레이아웃
layout = FloatLayout()

# 격자 위젯 생성 및 배치
grid_widget = GridVisualization(size_hint=(0.6, 0.6), pos_hint={'x': 0.3, 'y': 0.6})
layout.add_widget(grid_widget)

# ... (기존 right_box, bottom_box 등 추가)
