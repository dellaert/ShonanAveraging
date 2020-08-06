import React from 'react';
import Button from 'react-bootstrap/Button';
import ButtonGroup from 'react-bootstrap/ButtonGroup';

type Props = { values: Array<any>; setValue: any => void; current: any }

function ParameterGroup(props: Props) {
    return (
        <ButtonGroup className="mr-2">
            {props.values.map(value =>
                <Button onClick={() => props.setValue(value)}
                    key={value} active={value === props.current}
                    variant="outline-primary">{value}
                </Button>
            )}
        </ButtonGroup>)
}

export default ParameterGroup;