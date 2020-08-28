#include <boost/coroutine2/all.hpp>
#include <functional>
#include <boost/bind.hpp>
#include <iostream>

using VoidCoroutine = boost::coroutines2::coroutine<void>;
using TestFunction =
std::function<void(std::shared_ptr<int>, VoidCoroutine::push_type&)>;

class FunctionMemberObject
{
    public:
        explicit FunctionMemberObject(TestFunction func,
                                      std::shared_ptr<int> data) :
                func(func),
                coroutine_sequence(boost::bind(
                        &FunctionMemberObject::executeWrapper, this, _1, data))
        {
        }

        bool execute() {
            if (coroutine_sequence)
            {
                coroutine_sequence();
            }
            return !static_cast<bool>(coroutine_sequence);
        }

        private:
        void executeWrapper(VoidCoroutine::push_type& yield, std::shared_ptr<int> data) {
            yield();
            func(data, yield);
        }

        // Member variables are stored in the same memory (stack or heap) as
        // the allocated instance of this class. This is OUTSIDE the "coroutine stack"
        VoidCoroutine::pull_type coroutine_sequence;
        TestFunction func;
};

class DataMemberObject
{
public:
    explicit DataMemberObject(TestFunction func,
                                  std::shared_ptr<int> data) :
            data(data),
            coroutine_sequence(boost::bind(
                    &DataMemberObject::executeWrapper, this, _1, func))
    {
    }

    bool execute() {
        if (coroutine_sequence)
        {
            coroutine_sequence();
        }
        return !static_cast<bool>(coroutine_sequence);
    }

private:
    void executeWrapper(VoidCoroutine::push_type& yield, TestFunction func) {
        yield();
        func(data, yield);
    }

    // Member variables are stored in the same memory (stack or heap) as
    // the allocated instance of this class. This is OUTSIDE the "coroutine stack"
    VoidCoroutine::pull_type coroutine_sequence;
    std::shared_ptr<int> data;
};

class NoMemberObject
{
public:

    explicit NoMemberObject(TestFunction validation_function,
                                  std::shared_ptr<int> data) :
            coroutine_sequence(boost::bind(
                    &NoMemberObject::executeWrapper, this, _1, data, validation_function))
    {
    }

    bool execute() {
        if (coroutine_sequence)
        {
            coroutine_sequence();
        }
        return !static_cast<bool>(coroutine_sequence);
    }

private:
    void executeWrapper(VoidCoroutine::push_type& yield, std::shared_ptr<int> data, TestFunction func) {
        yield();
        func(data, yield);
    }

    VoidCoroutine::pull_type coroutine_sequence;
};


int main(int argc, char **argv) {
    auto print_data_func = [](std::shared_ptr<int> data, VoidCoroutine::push_type& yield) {
        while(true) {
            if(data) {
                std::cout << *data << std::endl;
            }else {
                std::cout << "null" << std::endl;
            }
            yield();
        }
    };
    auto data = std::make_shared<int>(99);

    {
        std::cout << "Copy and move semantics" << std::endl;

        // None of these lines compile because the copy constructor for the objects
        // is implicitly deleted by the coroutine. The copy constructors being deleted
        // makes sense since copying the coroutine would mean copying the "coroutine stack",
        // and that would get messy if the same stack existed in more than one place.

        FunctionMemberObject fmo_test(print_data_func, data);
        // Doesn't compile. Copy-constructor implicitly deleted by coroutine.
//    DataMemberObject fmo_test_copy = dmo_test;
        // Moving is ok
        FunctionMemberObject fmo_test_move = std::move(fmo_test);

        DataMemberObject dmo_test(print_data_func, data);
        // Doesn't compile. Copy-constructor implicitly deleted by coroutine.
//    DataMemberObject dmo_test_copy = dmo_test;
        // Moving is ok
        DataMemberObject dmo_test_move = std::move(dmo_test);

        NoMemberObject nmo_test(print_data_func, data);
        // Doesn't compile. Copy-constructor implicitly deleted by coroutine.
//    DataMemberObject fmo_test_copy = dmo_test;
        // Moving is ok
        NoMemberObject nmo_test_move = std::move(nmo_test);
    }

    {
        std::cout << std::endl << std::endl;
        std::cout << "Test on the stack with no move" << std::endl;

        FunctionMemberObject fmo(print_data_func, data);
        DataMemberObject dmo(print_data_func, data);
        NoMemberObject nmo(print_data_func, data);

        fmo.execute();
        dmo.execute();
        nmo.execute();

        *data = 5;

        fmo.execute();
        dmo.execute();
        nmo.execute();

        // Output
        // 99
        // 99
        // 99
        // 5
        // 5
        // 5
    }

    {
        std::cout << std::endl << std::endl;
        std::cout << "Test on the heap with no move" << std::endl;

        auto fmo_ptr = std::make_unique<FunctionMemberObject>(print_data_func, data);
        auto dmo_ptr = std::make_unique<DataMemberObject>(print_data_func, data);
        auto nmo_ptr = std::make_unique<NoMemberObject>(print_data_func, data);

        fmo_ptr->execute();
        dmo_ptr->execute();
        nmo_ptr->execute();

        *data = 8;

        fmo_ptr->execute();
        dmo_ptr->execute();
        nmo_ptr->execute();

        // Output
        // 5
        // 5
        // 5
        // 8
        // 8
        // 8
    }

    {
        std::cout << std::endl << std::endl;
        std::cout << "Test on the stack with moving variables" << std::endl;

        FunctionMemberObject fmo_test(print_data_func, data);
        DataMemberObject dmo_test(print_data_func, data);
        NoMemberObject nmo_test(print_data_func, data);

        fmo_test.execute();
        dmo_test.execute();
        nmo_test.execute();

        *data = -14;

        FunctionMemberObject fmo_test_move = std::move(fmo_test);
        DataMemberObject dmo_test_move = std::move(dmo_test);
        NoMemberObject nmo_test_move = std::move(nmo_test);

        fmo_test_move.execute();
        dmo_test_move.execute();
        nmo_test_move.execute();

        *data = -15;

        fmo_test_move.execute();
        dmo_test_move.execute();
        nmo_test_move.execute();
    }

    {
        std::cout << std::endl << std::endl;
        std::cout << "Test on the stack with moving vectors" << std::endl;

        std::vector<FunctionMemberObject> fmo_vector;
        fmo_vector.push_back(std::move(FunctionMemberObject(print_data_func, data)));
        std::vector<DataMemberObject> dmo_vector;
        dmo_vector.push_back(std::move(DataMemberObject(print_data_func, data)));
        std::vector<NoMemberObject> nmo_vector;
        nmo_vector.push_back(std::move(NoMemberObject(print_data_func, data)));

       // TODO: THIS BRWEAKS SHIT BELOW

        // Breaks things really bad. Either triggers std::bad_function_call,
        // or seems to mess up the program so badly it stops printing anything
//        fmo_vector.at(0).execute();
//        dmo_vector.at(0).execute();
//        nmo_vector.at(0).execute();

//        *data = -3;
//
//    fmo_vector.at(0).execute();
//        dmo_vector.at(0).execute();
//        nmo_vector.at(0).execute();

        // Output
        // std::bad_function_call
        // garbage / default-constructed data
        // 8
        // std::bad_function_call
        // garbage / default-constructed data
        // -3

        // Why does this not work for the DataMemberObject when it works
        // just moving variables??? In either case the variable should be
        // moved, so why is this different? The vector calling the
        // default constructor somewhere I'm not expecting?
    }

    {
        std::cout << std::endl << std::endl;
        std::cout << "Test on the heap with moving variables" << std::endl;

        auto fmo_ptr = std::make_unique<FunctionMemberObject>(print_data_func, data);
        auto dmo_ptr = std::make_unique<DataMemberObject>(print_data_func, data);
        auto nmo_ptr = std::make_unique<NoMemberObject>(print_data_func, data);

        fmo_ptr->execute();
        dmo_ptr->execute();
        nmo_ptr->execute();

        *data = 3;

        auto fmo_ptr_move = std::move(fmo_ptr);
        auto dmo_ptr_move = std::move(dmo_ptr);
        auto nmo_ptr_move = std::move(nmo_ptr);

        fmo_ptr_move->execute();
        dmo_ptr_move->execute();
        nmo_ptr_move->execute();

        *data = 4;

        fmo_ptr_move->execute();
        dmo_ptr_move->execute();
        nmo_ptr_move->execute();
    }

    {
        std::cout << std::endl << std::endl;
        std::cout << "Test on the heap with moving vectors" << std::endl;

        std::vector<std::unique_ptr<FunctionMemberObject>> fmo_vector;
        fmo_vector.push_back(std::make_unique<FunctionMemberObject>(print_data_func, data));
        std::vector<std::unique_ptr<DataMemberObject>> dmo_vector;
        dmo_vector.push_back(std::make_unique<DataMemberObject>(print_data_func, data));
        std::vector<std::unique_ptr<NoMemberObject>> nmo_vector;
        nmo_vector.push_back(std::make_unique<NoMemberObject>(print_data_func, data));

        fmo_vector.at(0)->execute();
        dmo_vector.at(0)->execute();
        nmo_vector.at(0)->execute();

        *data = 1;

        fmo_vector.at(0)->execute();
        dmo_vector.at(0)->execute();
        nmo_vector.at(0)->execute();
    }

    {
        std::cout << std::endl << std::endl;
        std::cout << "Test on the heap with moving vectors 2" << std::endl;

        std::vector<std::unique_ptr<FunctionMemberObject>> fmo_vector;
        auto fmo = std::make_unique<FunctionMemberObject>(print_data_func, data);
        std::vector<std::unique_ptr<DataMemberObject>> dmo_vector;
        auto dmo = std::make_unique<DataMemberObject>(print_data_func, data);
        std::vector<std::unique_ptr<NoMemberObject>> nmo_vector;
        auto nmo = std::make_unique<NoMemberObject>(print_data_func, data);

        fmo->execute();
        dmo->execute();
        nmo->execute();

        fmo_vector.push_back(std::move(fmo));
        dmo_vector.push_back(std::move(dmo));
        nmo_vector.push_back(std::move(nmo));

        *data = 7;

        fmo_vector.at(0)->execute();
        dmo_vector.at(0)->execute();
        nmo_vector.at(0)->execute();
    }

    return 0;
}
