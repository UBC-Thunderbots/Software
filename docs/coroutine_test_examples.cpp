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
              std::bind(&FunctionMemberObject::executeWrapper, this, std::placeholders::_1, data))
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

    // The TestFunction is stored in the same memory (stack or heap) as the allocated
    // instance of this class. The coroutine accesses this member variable directly
    // from "inside" the coroutine. On the other hand, the shared_ptr to the data
    // is passed into the coroutine right at the start.
    VoidCoroutine::pull_type coroutine_sequence;
    TestFunction func;
};

class DataMemberObject
{
   public:
    explicit DataMemberObject(TestFunction func, std::shared_ptr<int> data)
        : data(data),
          coroutine_sequence(
              std::bind(&DataMemberObject::executeWrapper, this, std::placeholders::_1, func))
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

    // The shared_ptr to the data is stored in the same memory (stack or heap) as the
    // allocated instance of this class. The coroutine accesses this member variable
    // directly from "inside" the coroutine. On the other hand, the TestFunction is passed
    // into the coroutine right at the start.
    VoidCoroutine::pull_type coroutine_sequence;
    std::shared_ptr<int> data;
};

class NoMemberObject
{
   public:
    explicit NoMemberObject(TestFunction validation_function, std::shared_ptr<int> data)
        : coroutine_sequence(std::bind(&NoMemberObject::executeWrapper, this, std::placeholders::_1, data,
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

    // This class passes the TestFunction and shared_ptr to the data into the
    // coroutine right at the start, so they are not stored in the same memory
    // as the allocated object.
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

void moveFromStackToStack()
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

void moveFromHeapToHeap()
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

void moveFromStackToHeapFMO()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test moving from stack to heap - FunctionMemberObject" << std::endl;

    auto data = std::make_shared<int>(99);

    FunctionMemberObject fmo(print_data_func, data);

    fmo.execute();

    *data = 45;

    auto fmo_heap = std::unique_ptr<FunctionMemberObject>();

    *fmo_heap = std::move(fmo);

    fmo_heap->execute();

    *data = 83;

    fmo_heap->execute();

    // Output
    // 99
    // (printouts stop working after this point)
}

void moveFromStackToHeapDMO()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test moving from stack to heap - DataMemberObject" << std::endl;

    auto data = std::make_shared<int>(99);

    DataMemberObject dmo(print_data_func, data);

    dmo.execute();

    *data = 45;

    auto dmo_heap = std::unique_ptr<DataMemberObject>();

    *dmo_heap = std::move(dmo);

    dmo_heap->execute();

    *data = 83;

    dmo_heap->execute();

    // Output
    // 99
    // (printouts stop working after this point)
}

void moveFromStackToHeapNMO()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test moving from stack to heap - NoMemberObject" << std::endl;

    auto data = std::make_shared<int>(99);

    NoMemberObject nmo(print_data_func, data);

    nmo.execute();

    *data = 45;

    auto nmo_heap = std::unique_ptr<NoMemberObject>();

    *nmo_heap = std::move(nmo);

    nmo_heap->execute();

    *data = 83;

    nmo_heap->execute();

    // Output
    // 99
    // (printouts stop working after this point)
}

void moveFromHeapToStack()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test moving from heap to stack" << std::endl;

    auto data = std::make_shared<int>(99);

    auto fmo_heap = std::make_unique<FunctionMemberObject>(print_data_func, data);
    auto dmo_heap = std::make_unique<DataMemberObject>(print_data_func, data);
    auto nmo_heap = std::make_unique<NoMemberObject>(print_data_func, data);

    std::optional<FunctionMemberObject> fmo;
    std::optional<DataMemberObject> dmo;
    std::optional<NoMemberObject> nmo;

    fmo_heap->execute();
    dmo_heap->execute();
    nmo_heap->execute();

    *data = 45;

    *fmo = std::move(*fmo_heap);
    *dmo = std::move(*dmo_heap);
    *nmo = std::move(*nmo_heap);

    fmo->execute();
    dmo->execute();
    nmo->execute();

    *data = 83;

    fmo->execute();
    dmo->execute();
    nmo->execute();

    // Output'
    // 99
    // 99
    // 99
    // 45
    // 45
    // 45
    // 83
    // 83
    // 83
}

void moveStackToVectorFMO()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test move stack variables into vector - FunctionMemberObject"
              << std::endl;

    auto data = std::make_shared<int>(99);

    std::vector<FunctionMemberObject> fmo_vector;
    fmo_vector.push_back(std::move(FunctionMemberObject(print_data_func, data)));

    fmo_vector.at(0).execute();

    *data = -3;

    fmo_vector.at(0).execute();

    // Output
    // std::bad_function_call / stops all printouts
    // std::bad_function_call / stops all printouts
}

void moveStackToVectorDMO()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test move stack variables into vector - DataMemberObject" << std::endl;

    auto data = std::make_shared<int>(99);

    std::vector<DataMemberObject> dmo_vector;
    dmo_vector.push_back(std::move(DataMemberObject(print_data_func, data)));

    dmo_vector.at(0).execute();

    *data = -3;

    dmo_vector.at(0).execute();

    // Output
    // null
    // null
}

void moveStackToVectorNMO()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test move stack variables into vector - NoMemberObject" << std::endl;

    auto data = std::make_shared<int>(99);

    std::vector<NoMemberObject> nmo_vector;
    nmo_vector.push_back(std::move(NoMemberObject(print_data_func, data)));

    nmo_vector.at(0).execute();

    *data = -3;

    nmo_vector.at(0).execute();

    // Output
    // 99
    // -3
}

void moveHeapToVector()
{
    std::cout << std::endl << std::endl;
    std::cout << "Test move heap variables into vector" << std::endl;

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
    // We can verify this with the code below, or by looking at pull_coroutine.hpp
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


    // ~~~~~~~~~~ Experiments and Observations ~~~~~~~~~~ //
    // Note: Run each of these functions independently
    // since failures from one can affect the behaviour
    // of later functions

    /* There are no issues if no data is moved. This makes sense
     * since this is just a basic usage of coroutines, and is a
     * case we really expect to work.
     */
    //    variablesOnStackNoMove();           // OK
    //    variablesOnHeapNoMove();            // OK

    /* Moving coroutines works fine as long as the memory
     * is moved from the stack to the stack, of from the
     * heap to the heap
     */
    //    moveFromStackToStack();         // OK
    //    moveFromHeapToHeap();          // OK

    /* For some reason, moving coroutines from the stack to the heap
     * does not work in both directions. We don't have any ideas
     * why moving from the stack to the heap causes issues, while
     * moving from the heap to the stack does not.
     */
    //    moveFromStackToHeapFMO(); // BAD
    //    moveFromStackToHeapDMO(); // BAD
    //    moveFromStackToHeapNMO(); // BAD
    //    moveFromHeapToStack(); // OK

    /* Because vectors (and other resizable containers) actually store
     * data on the heap, moving coroutines into vectors only works
     * in certain cases.
     *
     * As seen above, moving data from the heap to the heap works fine,
     * and is also the case here when the data starts on the heap.
     *
     * We are not sure why moving data from the stack to the vector heap
     * does not work here, when it seems to work fine in the above test
     * without vectors. Specifically, it fails with the FunctionMemberObject
     * and DataMemberObject, but works fine with NoMemberObject.
     */
    //    moveStackToVectorFMO();   // BAD
    //    moveStackToVectorDMO();   // BAD
    //    moveStackToVectorNMO();   // OK
    //    moveHeapToVector();   // OK


    // ~~~~~~~~~~ Conclusions ~~~~~~~~~~ //
    /* - Whether a resizable container is used or not, moving coroutines
     *   between the stack and the heap results in inconsistent behaviour
     *   (it works in some cases but not others, without any obviously
     *   consistent rules). Therefore, coroutines should never be
     *   moved between the stack and heap in any way.
     *
     * - In some "bad" cases, such as moveStackToVector(), the NoMemberObject
     *   actually works fine. This seems to imply that coroutines behave better,
     *   especially when being moved, when any data it's accessing is
     *   passed into the coroutine when it's created so it exists on
     *   the "coroutine stack". Therefore, prefer passing as much data
     *   to the coroutine as possible on creation and avoid using member
     *   variables when possible.
     *
     * COROUTINES BEST PRACTICES
     * - Avoid moving coroutines. If the absolutely must be moved,
     *   make sure they are not moved between the stack and heap.
     * - Avoid using coroutines with resizable containers. If they
     *   must be used, make sure that the coroutines are allocated
     *   on the heap.
     * - Pass data to the coroutine on creation as much as possible,
     *   avoid using member variables.
     */

    return 0;
}
