package org.firstinspires.ftc.ramsrobotics.util

import java.util.Objects
import java.util.function.Consumer
import java.util.function.Function
import java.util.function.Supplier

object Optionull {
    fun <T> orElse(t: T?, defaultValue: T?): T? {
        return Objects.requireNonNullElse<T?>(t, defaultValue)
    }

    fun <T> ifPresent(t: T?, consumer: Consumer<T?>) {
        if (t != null) {
            consumer.accept(t)
        }
    }

    fun <T, R> map(t: T?, map: Function<T?, R?>): R? {
        return if (t == null) null else map.apply(t)
    }

    fun <T, R> mapOrDefault(t: T?, map: Function<T?, R?>, defaultValue: R?): R? {
        return if (t == null) defaultValue else map.apply(t)
    }

    fun <T, R> mapOrElse(t: T?, map: Function<T?, R?>, elseSupplier: Supplier<R?>): R? {
        return if (t == null) elseSupplier.get() else map.apply(t)
    }

    fun <T> first(collection: MutableCollection<T?>): T? {
        val iterator = collection.iterator()
        return if (iterator.hasNext()) iterator.next() else null
    }

    fun <T> firstOrDefault(collection: MutableCollection<T?>, defaultValue: T?): T? {
        val iterator = collection.iterator()
        return if (iterator.hasNext()) iterator.next() else defaultValue
    }

    fun <T> firstOrElse(collection: MutableCollection<T?>, elseSupplier: Supplier<T?>): T? {
        val iterator = collection.iterator()
        return if (iterator.hasNext()) iterator.next() else elseSupplier.get()
    }

    fun <T> isNullOrEmpty(t: Array<T?>?): Boolean {
        return t == null || t.size == 0
    }

    fun isNullOrEmpty(t: BooleanArray?): Boolean {
        return t == null || t.size == 0
    }

    fun isNullOrEmpty(t: ByteArray?): Boolean {
        return t == null || t.size == 0
    }

    fun isNullOrEmpty(t: CharArray?): Boolean {
        return t == null || t.size == 0
    }

    fun isNullOrEmpty(t: ShortArray?): Boolean {
        return t == null || t.size == 0
    }

    fun isNullOrEmpty(t: IntArray?): Boolean {
        return t == null || t.size == 0
    }

    fun isNullOrEmpty(t: LongArray?): Boolean {
        return t == null || t.size == 0
    }

    fun isNullOrEmpty(t: FloatArray?): Boolean {
        return t == null || t.size == 0
    }

    fun isNullOrEmpty(t: DoubleArray?): Boolean {
        return t == null || t.size == 0
    }
}