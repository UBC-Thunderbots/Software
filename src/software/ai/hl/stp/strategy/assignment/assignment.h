MAKE_ENUM(AssignmentType, OFFENSIVE, MIDFIELD, DEFENSIVE)

class Assignment
{
    public:
        virtual AssignmentType getAssignmentType() const = 0;
};
