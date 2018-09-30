import * as React from 'react';
import { Subscribe } from 'unstated';
import { TopicContainer, TopicContainerType } from '../../components/containers/rosTopic';

export const Logger = () => {
    return (
        <Subscribe to={[TopicContainer('/rosout')]}>
            {(rosout: TopicContainerType) => (
                <div>
                    {rosout.state.values.map((element) => (
                        <div>
                            {element.msg}
                        </div>
                    ))}
                </div>
            )}
        </Subscribe>
    )
}