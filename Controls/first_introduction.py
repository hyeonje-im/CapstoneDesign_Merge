 # components/right_items.py
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label

def introductions():
    items = [
        ("Obstacle rearrangement", "Relocate the grid obstacles and manually define the accessible area."),
        ("Robot's goal positions setting", "Relocate the grid obstacles and manually define the accessible area."),
        ("Low-level robot control", "Try manually controlling the robot's start and stop times."),
        
    ]

    item_widgets = []

    for title, desc in items:
        item_box = BoxLayout(orientation='vertical', size_hint_y=0.5)
        title_label = Label(text=f"[b]{title}[/b]", markup=True, color=(1, 1, 1, 1), halign='left', valign='middle')
        desc_label = Label(text=desc, color=(1, 1, 1, 0.7), halign='left', valign='top')

        title_label.bind(size=lambda inst, *a: setattr(inst, 'text_size', inst.size))
        desc_label.bind(size=lambda inst, *a: setattr(inst, 'text_size', inst.size))

        item_box.add_widget(title_label)
        item_box.add_widget(desc_label)
        item_widgets.append(item_box)

    return item_widgets
