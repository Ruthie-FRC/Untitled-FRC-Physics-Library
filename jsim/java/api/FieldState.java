package jsim.api;

public class FieldState<T> {
    private T state;
    public FieldState(T state) {
        this.state = state;
    }
    public T getState() {
        return state;
    }
    public void setState(T state) {
        this.state = state;
    }
}
