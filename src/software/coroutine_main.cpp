#include <boost/coroutine2/all.hpp>
#include <functional>
#include <boost/bind.hpp>
#include <iostream>

using ValidationCoroutine = boost::coroutines2::coroutine<void>;
using ValidationFunction =
std::function<void(std::shared_ptr<int>, ValidationCoroutine::push_type&)>;

class FunctionValidator
{
        public:

        explicit FunctionValidator(ValidationFunction validation_function,
                std::shared_ptr<int> data) :
                validation_function(validation_function),
                validation_sequence(boost::bind(
                &FunctionValidator::executeAndCheckForSuccessWrapper, this, _1, data))
        {
        }

        bool executeAndCheckForSuccess() {
            // Check the coroutine status to see if it has any more work to do.
                if (validation_sequence)
                {
                        // Run the coroutine. This will call the bound executeAndCheckForSuccessWrapper
                        // function
                        validation_sequence();
                }
            // The validation_function is done if the coroutine evaluates to false, which means
            // execution has "dropped out" the bottom of the function and there is no more work to
            // do. If this is the case then the validation_function has passed successfully
            return !static_cast<bool>(validation_sequence);
        }

        private:
        void executeAndCheckForSuccessWrapper(ValidationCoroutine::push_type& yield, std::shared_ptr<int> data) {
            // Yield the very first time the function is called, so that the validation_function
            // is not run until this coroutine / wrapper function is called again by
            // executeAndCheckForSuccess
            yield();
            // Anytime after the first function call, the validation_function will be
            // used to perform the real logic.
            validation_function(data, yield);
        }

        ValidationCoroutine::pull_type validation_sequence;
        ValidationFunction validation_function;
};


int main(int argc, char **argv) {
    auto func = [](std::shared_ptr<int> data, ValidationCoroutine::push_type& yield) {
        while(true) {
            if(data) {
                std::cout << *data << std::endl;
            }else {
                std::cout << "null" << std::endl;
            }
            yield();
        }
    };
    auto data = std::make_shared<int>(0);
    FunctionValidator fv(func, data);
    fv.executeAndCheckForSuccess();
    *data = 1;
    fv.executeAndCheckForSuccess();
    *data = 4;
    fv.executeAndCheckForSuccess();
    return 0;
}
