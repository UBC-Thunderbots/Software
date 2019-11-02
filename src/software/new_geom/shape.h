class IShape
{
public:
    virtual ~IShape() = default;

    virtual double area() const = 0;
};
