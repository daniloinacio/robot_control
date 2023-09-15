import py_trees.common

# Define some behavior tree nodes
class ActionNode(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        print(f"Performing action: {self.name}")
        return py_trees.common.Status.SUCCESS

# Create the behavior tree
root = py_trees.composites.Selector("Root", False)
action = ActionNode("Action 1")
inverter = py_trees.decorators.Inverter(child=action, name="Inverter 1")
root.add_child(inverter)
sequence = py_trees.composites.Sequence(name="Sequence", memory=False)
sequence.add_child(ActionNode("Action 2"))
sequence.add_child(ActionNode("Action 3"))
root.add_child(sequence)

# Create the behavior tree manager and tick the tree
tree = py_trees.trees.BehaviourTree(root)
tree.setup(timeout=1)
tree.tick_tock(
    period_ms=50,
    # number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
    number_of_iterations=10,
    pre_tick_handler=None,
    post_tick_handler=None
)
py_trees.display.render_dot_tree(root)