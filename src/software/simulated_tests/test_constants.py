import software.python_bindings as tbots

DIV_B_FIELD = tbots.Field.createSSLDivisionBField()
FIELD_X_LENGTH = DIV_B_FIELD.xLength()
FIELD_Y_LENGTH = DIV_B_FIELD.yLength()
FRIENDLY_DEFENSE_AREA = DIV_B_FIELD.friendlyDefenseArea()
FRIENDLY_GOAL_AREA = DIV_B_FIELD.friendlyGoal()

FIELD_RIGHT_HALF = tbots.Rectangle(
    tbots.Point(0, -FIELD_Y_LENGTH / 2),
    tbots.Point(FIELD_X_LENGTH / 2 + FRIENDLY_GOAL_AREA.xLength(), FIELD_Y_LENGTH / 2),
)
