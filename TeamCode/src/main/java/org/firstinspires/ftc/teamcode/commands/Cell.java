package org.firstinspires.ftc.teamcode.commands;

public class Cell<T> {
    private T value;

    public Cell(T initialValue) {
        this.value = initialValue;
    }

    public T get() {
        return value;
    }

    public void set(T value) {
        this.value = value;
    }

    public void map(java.util.function.UnaryOperator<T> mapper) {
        this.value = mapper.apply(this.value);
    }
}