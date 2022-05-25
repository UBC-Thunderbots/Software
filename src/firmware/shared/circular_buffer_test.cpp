extern "C"
{
#include "firmware/shared/circular_buffer.h"

#include "shared/constants.h"
}

#include <gtest/gtest.h>

class CircularBufferTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        circular_buffer = circular_buffer_create(10);
    }

    virtual void TearDown(void)
    {
        circular_buffer_destroy(circular_buffer);
    }

    CircularBuffer_t* circular_buffer;
};

TEST(CircularBufferInit0Test, circular_buffer_init_0)
{
    ASSERT_DEATH(circular_buffer_create(0), "size > 0");
}

TEST_F(CircularBufferTest, circular_buffer_exact_full)
{
    float mock_data[]     = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    float expected_data[] = {10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
    int mock_data_size    = sizeof(mock_data) / sizeof(mock_data[0]);

    EXPECT_TRUE(circular_buffer_isEmpty(circular_buffer));

    // Populate circular buffer with mock data
    for (int i = 0; i < mock_data_size; i++)
    {
        circular_buffer_push(circular_buffer, mock_data[i]);
    }

    EXPECT_TRUE(circular_buffer_isFull(circular_buffer));

    for (int i = 0; i < mock_data_size; i++)
    {
        EXPECT_EQ(expected_data[i], circular_buffer_getAtIndex(circular_buffer, i));
    }
}

TEST_F(CircularBufferTest, circular_buffer_overfill)
{
    float mock_data[]      = {1.0, 2.0,  3.0,  4.0,  5.0,  6.0,  7.0, 8.0,
                         9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0};
    float expected_data[]  = {15.0, 14.0, 13.0, 12.0, 11.0, 10.0, 9.0, 8.0, 7.0, 6.0};
    int mock_data_size     = sizeof(mock_data) / sizeof(mock_data[0]);
    int expected_data_size = sizeof(expected_data) / sizeof(expected_data[0]);

    EXPECT_TRUE(circular_buffer_isEmpty(circular_buffer));

    // Populate circular buffer with mock data
    for (int i = 0; i < mock_data_size; i++)
    {
        circular_buffer_push(circular_buffer, mock_data[i]);
    }

    EXPECT_TRUE(circular_buffer_isFull(circular_buffer));

    for (int i = 0; i < expected_data_size; i++)
    {
        EXPECT_EQ(expected_data[i], circular_buffer_getAtIndex(circular_buffer, i));
    }
}

TEST_F(CircularBufferTest, circular_buffer_not_full)
{
    float mock_data[]     = {1.0, 2.0, 3.0, 4.0, 5.0};
    float expected_data[] = {5.0, 4.0, 3.0, 2.0, 1.0};
    int mock_data_size    = sizeof(mock_data) / sizeof(mock_data[0]);

    EXPECT_TRUE(circular_buffer_isEmpty(circular_buffer));

    // Populate circular buffer with mock data
    for (int i = 0; i < mock_data_size; i++)
    {
        circular_buffer_push(circular_buffer, mock_data[i]);
    }

    EXPECT_FALSE(circular_buffer_isFull(circular_buffer));
    EXPECT_FALSE(circular_buffer_isEmpty(circular_buffer));

    for (int i = 0; i < mock_data_size; i++)
    {
        EXPECT_EQ(expected_data[i], circular_buffer_getAtIndex(circular_buffer, i));
    }
}

TEST_F(CircularBufferTest, circular_buffer_get_at_index_larger_buffer_size)
{
    float mock_data[]  = {1.0, 2.0, 3.0};
    int mock_data_size = sizeof(mock_data) / sizeof(mock_data[0]);

    EXPECT_TRUE(circular_buffer_isEmpty(circular_buffer));

    // Populate circular buffer with mock data
    for (int i = 0; i < mock_data_size; i++)
    {
        circular_buffer_push(circular_buffer, mock_data[i]);
    }

    EXPECT_FALSE(circular_buffer_isFull(circular_buffer));
    EXPECT_FALSE(circular_buffer_isEmpty(circular_buffer));

    ASSERT_DEATH(circular_buffer_getAtIndex(circular_buffer, 12);
                 , "index <= cbuffer->max_size");
}

TEST_F(CircularBufferTest, circular_buffer_get_at_index_unpopulated_not_full)
{
    float mock_data[]  = {1.0, 2.0, 3.0};
    int mock_data_size = sizeof(mock_data) / sizeof(mock_data[0]);

    EXPECT_TRUE(circular_buffer_isEmpty(circular_buffer));

    // Populate circular buffer with mock data
    for (int i = 0; i < mock_data_size; i++)
    {
        circular_buffer_push(circular_buffer, mock_data[i]);
    }

    EXPECT_FALSE(circular_buffer_isFull(circular_buffer));
    EXPECT_FALSE(circular_buffer_isEmpty(circular_buffer));

    ASSERT_DEATH(circular_buffer_getAtIndex(circular_buffer, 5);
                 , "circular_buffer_isFull(cbuffer) == true || index <= cbuffer->head");
}

TEST_F(CircularBufferTest, circular_buffer_front_exact_full)
{
    float mock_data[]   = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    int mock_data_size  = sizeof(mock_data) / sizeof(mock_data[0]);
    float expected_data = 10.0;

    EXPECT_TRUE(circular_buffer_isEmpty(circular_buffer));

    // Populate circular buffer with mock data
    for (int i = 0; i < mock_data_size; i++)
    {
        circular_buffer_push(circular_buffer, mock_data[i]);
    }

    EXPECT_TRUE(circular_buffer_isFull(circular_buffer));
    EXPECT_FALSE(circular_buffer_isEmpty(circular_buffer));

    EXPECT_EQ(expected_data, circular_buffer_front(circular_buffer));
}

TEST_F(CircularBufferTest, circular_buffer_front_overfill)
{
    float mock_data[] = {1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,  8.0,  9.0,  10.0, 11.0,
                         12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0};
    int mock_data_size  = sizeof(mock_data) / sizeof(mock_data[0]);
    float expected_data = 21.0;

    EXPECT_TRUE(circular_buffer_isEmpty(circular_buffer));

    // Populate circular buffer with mock data
    for (int i = 0; i < mock_data_size; i++)
    {
        circular_buffer_push(circular_buffer, mock_data[i]);
    }

    EXPECT_TRUE(circular_buffer_isFull(circular_buffer));
    EXPECT_FALSE(circular_buffer_isEmpty(circular_buffer));

    EXPECT_EQ(expected_data, circular_buffer_front(circular_buffer));
}

TEST_F(CircularBufferTest, circular_buffer_front_not_full)
{
    float mock_data[]   = {1.0, 2.0, 3.0, 4.0, 5.0};
    int mock_data_size  = sizeof(mock_data) / sizeof(mock_data[0]);
    float expected_data = 5.0;

    EXPECT_TRUE(circular_buffer_isEmpty(circular_buffer));

    circular_buffer_push(circular_buffer, mock_data[0]);

    // Populate rest of circular buffer with mock data
    for (int i = 1; i < mock_data_size; i++)
    {
        circular_buffer_push(circular_buffer, mock_data[i]);
        EXPECT_FALSE(circular_buffer_isFull(circular_buffer));
        EXPECT_FALSE(circular_buffer_isEmpty(circular_buffer));
    }

    EXPECT_FALSE(circular_buffer_isFull(circular_buffer));
    EXPECT_FALSE(circular_buffer_isEmpty(circular_buffer));

    EXPECT_EQ(expected_data, circular_buffer_front(circular_buffer));
}

TEST_F(CircularBufferTest, circular_buffer_front_empty)
{
    EXPECT_FALSE(circular_buffer_isFull(circular_buffer));
    EXPECT_TRUE(circular_buffer_isEmpty(circular_buffer));

    ASSERT_DEATH(circular_buffer_front(circular_buffer);
                 , "circular_buffer_isFull(cbuffer) == true || cbuffer->head > 0");
}
