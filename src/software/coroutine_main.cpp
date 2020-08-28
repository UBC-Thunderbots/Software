#include <boost/bind.hpp>
#include <boost/coroutine2/all.hpp>
#include <functional>
#include <iostream>

using VoidCoroutine = boost::coroutines2::coroutine<void>;
using TestFunction = std::function<void(std::shared_ptr<int>, VoidCoroutine::push_type&)>;

class FunctionMemberObject
{
   public:
    explicit FunctionMemberObject(TestFunction func, std::shared_ptr<int> data)
        : func(func),
          coroutine_sequence(
              boost::bind(&FunctionMemberObject::executeWrapper, this, _1, data))
    {
    }

    bool execute()
    {
        if (coroutine_sequence)
        {
            coroutine_sequence();
        }
        return !static_cast<bool>(coroutine_sequence);
    }

   private:
    void executeWrapper(VoidCoroutine::push_type& yield, std::shared_ptr<int> data)
    {
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
    explicit DataMemberObject(TestFunction func, std::shared_ptr<int> data)
        : data(data),
          coroutine_sequence(
              boost::bind(&DataMemberObject::executeWrapper, this, _1, func))
    {
    }

    bool execute()
    {
        if (coroutine_sequence)
        {
            coroutine_sequence();
        }
        return !static_cast<bool>(coroutine_sequence);
    }

   private:
    void executeWrapper(VoidCoroutine::push_type& yield, TestFunction func)
    {
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
    explicit NoMemberObject(TestFunction validation_function, std::shared_ptr<int> data)
        : coroutine_sequence(boost::bind(&NoMemberObject::executeWrapper, this, _1, data,
                                         validation_function))
    {
    }

    bool execute()
    {
        if (coroutine_sequence)
        {
            coroutine_sequence();
        }
        return !static_cast<bool>(coroutine_sequence);
    }

   private:
    void executeWrapper(VoidCoroutine::push_type& yield, std::shared_ptr<int> data,
                        TestFunction func)
    {
        yield();
        func(data, yield);
    }

    VoidCoroutine::pull_type coroutine_sequence;
};

auto print_data_func = [](std::shared_ptr<int> data, VoidCoroutine::push_type& yield) {
    while (true)
    {
        if (data)
        {
            std::cout << *data << std::endl;
        }
        else
        {
            std::cout << "null" << std::endl;
        }
        yield();
    }
};

void variablesOnStackNoMove()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test on the stack with no move" << std::endl;

    auto data = std::make_shared<int>(99);

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

void variablesOnHeapNoMove()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test on the heap with no move" << std::endl;

    auto data = std::make_shared<int>(99);

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
    // 99
    // 99
    // 99
    // 8
    // 8
    // 8
}

void variablesOnStackWithMove()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test on the stack with moving variables" << std::endl;

    auto data = std::make_shared<int>(99);

    FunctionMemberObject fmo_test(print_data_func, data);
    DataMemberObject dmo_test(print_data_func, data);
    NoMemberObject nmo_test(print_data_func, data);

    fmo_test.execute();
    dmo_test.execute();
    nmo_test.execute();

    *data = -14;

    FunctionMemberObject fmo_test_move = std::move(fmo_test);
    DataMemberObject dmo_test_move     = std::move(dmo_test);
    NoMemberObject nmo_test_move       = std::move(nmo_test);

    fmo_test_move.execute();
    dmo_test_move.execute();
    nmo_test_move.execute();

    *data = -15;

    fmo_test_move.execute();
    dmo_test_move.execute();
    nmo_test_move.execute();

    // Output
    // 99
    // 99
    // 99
    // -14
    // -14
    // -14
    // -15
    // -15
    // -15
}

void variablesOnStackWithMoveVector()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test on the stack with moving vectors" << std::endl;

    auto data = std::make_shared<int>(99);

    std::vector<FunctionMemberObject> fmo_vector;
    fmo_vector.push_back(std::move(FunctionMemberObject(print_data_func, data)));
    std::vector<DataMemberObject> dmo_vector;
    dmo_vector.push_back(std::move(DataMemberObject(print_data_func, data)));
    std::vector<NoMemberObject> nmo_vector;
    nmo_vector.push_back(std::move(NoMemberObject(print_data_func, data)));

    //    fmo_vector.at(0).execute();
    //    dmo_vector.at(0).execute();
    nmo_vector.at(0).execute();

    *data = -3;

    //    fmo_vector.at(0).execute();
    //    dmo_vector.at(0).execute();
    nmo_vector.at(0).execute();

    // Output
    // std::bad_function_call / stops all printouts
    // 0 (seems to be default-constructed data)
    // 99
    // std::bad_function_call / stops all printouts
    // 0 (seems to be default-constructed data)
    // -3

    // Why does this not work for the DataMemberObject when it works
    // just moving variables??? In either case the variable should be
    // moved, so why is this different? The vector calling the
    // default constructor somewhere I'm not expecting?
}

void variablesOnHeapWithMove()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test on the heap with moving variables" << std::endl;

    auto data = std::make_shared<int>(99);

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

    // Output
    // 99
    // 99
    // 99
    // 3
    // 3
    // 3
    // 4
    // 4
    // 4
}

void variablesOnHeapWithMoveVectors()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test on the heap with moving vectors" << std::endl;

    auto data = std::make_shared<int>(99);

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

    // Output
    // 99
    // 99
    // 99
    // 1
    // 1
    // 1
}

void variablesOnHeapWithMoveVectors2()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test on the heap with moving vectors 2" << std::endl;

    auto data = std::make_shared<int>(99);

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

    // Output
    // 99
    // 99
    // 99
    // 7
    // 7
    // 7
}

int main(int argc, char** argv)
{
    // ~~~~~~~~~~ Copy and Move semantics ~~~~~~~~~~
    // The copy-constructor is deleted for VoidCoroutine::pull_type, which
    // implicitly deletes it for all classes that use it. The copy constructors
    // being deleted makes sense since copying the coroutine would mean copying
    // the "coroutine stack", and that would get messy if the same stack existed
    // in more than one place.
    //
    // The move constructor / assgnment is allowed.
    //
    // We can verify this with the code below, or by lookinAg at pull_coroutine.hpp
    // https://www.boost.org/doc/libs/1_74_0/libs/coroutine2/doc/html/coroutine2/coroutine/asymmetric/pull_coro.html

    auto data = std::make_shared<int>(99);
    FunctionMemberObject fmo(print_data_func, data);
    DataMemberObject dmo(print_data_func, data);
    NoMemberObject nmo(print_data_func, data);

    // Won't compile
    //    FunctionMemberObject fmo_copy = fmo;
    //    DataMemberObject dmo_copy = dmo;
    //    NoMemberObject nmo_copy = nmo;

    // OK
    FunctionMemberObject fmo_move = std::move(fmo);
    DataMemberObject dmo_move     = std::move(dmo);
    NoMemberObject nmo_move       = std::move(nmo);


    // ~~~~~~~~~~ Experiments and Examples ~~~~~~~~~~ //
    variablesOnStackNoMove();           // OK
    variablesOnHeapNoMove();            // OK
    variablesOnStackWithMove();         // OK
    variablesOnStackWithMoveVector();   // BAD
    variablesOnHeapWithMove();          // OK
    variablesOnHeapWithMoveVectors();   // OK
    variablesOnHeapWithMoveVectors2();  // OK

    // ~~~~~~~~~~ Observations and Conclusions ~~~~~~~~~~ //
    /* - The "coroutine stack" seems to be able to read from and interact
     *   with the "normal stack" just fine as long as the objects do not move.
     *   This goes for when the objects are created on the stack or heap.
     *
     * - When objects are created on the stack, moving single variables
     *   seems to keep everything working fine. However, moving into / out of
     *   containers (like vectors) seems to mess things up. We don't know exactly
     *   why containers cause things to behave differently. It could be that the
     *   implementation of the move constructor/assignment isn't robust enough.
     *
     * - When objects are created on the heap, moving single variables
     *   as well as moving containers works fine. This implies that the
     *   "coroutine stack" seems to work better when it's not closely interacting
     *   with another "normal stack". Things generally behave better when the
     *   object containing the coroutine is allocated on the heap.
     *
     * - In all cases, avoiding storing ANY variables (other than the coroutine
     *   object itself) on the stack (ie. the NoMemberObject) worked fine. So it
     *   seems that as long as any variables that need to be accessed / used
     *   "inside" the coroutine are not stored on the "normal stack"
     *   (they can be on the heap or passed into the coroutine directly),
     *   things will behave as expected.
     *
     *   tl;dr
     *
     * - Things work fine if you don't move them
     * - Things work fine if the objects containing coroutines are
     *   allocated on the heap
     * - Things work fine if all values are passed directly into
     *   the coroutine (ie. the NoMemberObject).
     *   This is generally the best option.
     */

    return 0;
}
